# arduino_interface/arduino_interface/position_tracker_node.py
# (센서 퓨전: 가속도계 바이어스 보정을 통한 위치 추적 버전)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
import time

class PositionTrackerNode(Node):
    def __init__(self):
        super().__init__('position_tracker_node')
        
        self.subscription = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/tracked_pose', 10)
        self.path_publisher = self.create_publisher(Path, '/trajectory/path', 10)
        
        # --- 상태 변수 ---
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_velocity = np.array([0.0, 0.0, 0.0])
        self.last_time = None

        # =======================================================================
        # ===> 드리프트 억제를 위한 튜닝 파라미터 <===
        # =======================================================================
        # 속도 감쇠: 1.0에 가까울수록 적분 값에 가깝고, 작을수록 드리프트 억제력이 강해집니다.
        # 너무 작으면 움직임이 둔감해집니다.
        self.velocity_damping = 1.0
        
        # 중력 상수
        self.gravity = 9.80665
        # =======================================================================

        # --- 가속도계 바이어스 보정 관련 변수 ---
        self.calibration_duration = 3.0  # 3초간 보정
        self.calibration_end_time = time.time() + self.calibration_duration
        self.is_calibrating = True
        self.accel_bias_samples = []
        self.accelerometer_bias = np.array([0.0, 0.0, 0.0])

        # 경로 저장을 위한 변수
        self.path_msg = Path()
        self.path_msg.header.frame_id = "world"
        self.max_path_size = 500
        
        self.get_logger().info("Position Tracker Node has started.")
        self.get_logger().info(f"Keep the IMU stationary for {self.calibration_duration} seconds for accelerometer calibration...")

    def calibrate_accelerometer(self, msg):
        """시작 시 가속도계 바이어스를 측정하는 함수"""
        if time.time() < self.calibration_end_time:
            accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
            self.accel_bias_samples.append(accel)
        else:
            # 보정 시간이 끝나면 평균을 내어 바이어스로 저장
            if self.accel_bias_samples:
                self.accelerometer_bias = np.mean(self.accel_bias_samples, axis=0)
                # 바이어스에서 중력 성분은 제거해야 함 (z축 기준)
                # 실제로는 IMU의 초기 자세에 따라 달라지지만, 평평하게 놓였다고 가정
                self.accelerometer_bias[2] -= self.gravity 
            
            self.is_calibrating = False
            self.last_time = self.get_clock().now() # 보정이 끝난 시점부터 시간 측정 시작
            self.get_logger().info(f"Calibration complete. Accelerometer bias: {self.accelerometer_bias}")

    def imu_callback(self, msg):
        # 1. 보정 단계 처리
        if self.is_calibrating:
            self.calibrate_accelerometer(msg)
            return

        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return
        
        # 시간 변화량(dt) 계산
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0: return # 시간 간격이 매우 짧으면 무시
        self.last_time = now

        # 2. 현재 자세와 측정된 가속도 가져오기
        orientation_q = msg.orientation
        ros_q = np.array([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        
        # 측정된 가속도에서 계산된 바이어스 제거
        measured_acceleration = np.array([
            msg.linear_acceleration.x - self.accelerometer_bias[0],
            msg.linear_acceleration.y - self.accelerometer_bias[1],
            msg.linear_acceleration.z - self.accelerometer_bias[2]
        ])
        
        # 쿼터니언 유효성 확인
        if abs(np.linalg.norm(ros_q) - 1.0) > 0.1:
            self.get_logger().warn("Received non-normalized quaternion, skipping.", throttle_duration_sec=1.0)
            return
            
        current_rotation = R.from_quat(ros_q)

        # 3. 중력 보상: 측정된 가속도에서 중력 성분을 빼서 '순수 이동 가속도'를 얻음
        gravity_vector_world = np.array([0, 0, -self.gravity])
        gravity_in_imu_frame = current_rotation.inv().apply(gravity_vector_world)
        true_linear_acceleration = measured_acceleration - gravity_in_imu_frame
        
        # 4. 순수 이동 가속도를 월드 좌표계 기준으로 변환
        true_linear_acceleration_world = current_rotation.apply(true_linear_acceleration)

        true_linear_acceleration_world[2] = 0.0

        # 5. 속도 및 위치 업데이트 (이중 적분)
        # 5a. 속도 감쇠 적용 (드리프트 억제)
        self.current_velocity *= self.velocity_damping
        
        # 5b. 가속도를 적분하여 속도에 더함
        self.current_velocity += true_linear_acceleration_world * dt
        
        # 5c. 속도를 적분하여 위치에 더함
        self.current_position += self.current_velocity * dt
        
        # 6. 목표 좌표(PoseStamped) 메시지 생성 및 발행
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now.to_msg()
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = self.current_position[0]
        pose_msg.pose.position.y = self.current_position[1]
        pose_msg.pose.position.z = self.current_position[2]
        
        # 위치만 추적하므로 방향은 기본값으로 설정
        pose_msg.pose.orientation.w = 1.0 
        self.pose_publisher.publish(pose_msg)

        # 7. 궤적(Path) 메시지 업데이트 및 발행
        self.path_msg.header.stamp = now.to_msg()
        self.path_msg.poses.append(pose_msg)
        if len(self.path_msg.poses) > self.max_path_size:
            self.path_msg.poses.pop(0)
        self.path_publisher.publish(self.path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PositionTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
