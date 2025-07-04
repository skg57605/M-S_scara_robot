# arduino_interface/arduino_interface/orientation_to_position_node.py
# (센서 퓨전: 중력 가속도 보상 및 안정화 튜닝 파라미터가 적용된 최종 버전)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np
from scipy.spatial.transform import Rotation as R

class OrientationToPositionNode(Node):
    def __init__(self):
        super().__init__('orientation_to_position_node')
        
        self.subscription = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.path_publisher = self.create_publisher(Path, '/trajectory/path', 10)
        
        # --- 상태 변수 ---
        self.home_position = np.array([0.25, 0.0, 0.0]) # 로봇의 초기 홈 위치
        self.current_position = np.array([0.25, 0.0, 0.0])
        self.current_velocity = np.array([0.0, 0.0, 0.0])
        self.last_time = None

        # =======================================================================
        # ===> 안정화를 위한 튜닝 파라미터 (이 값들을 조절하여 제어감을 찾으세요!) <===
        # =======================================================================
        
        # 1. 민감도: 가속도 신호를 얼마나 크게 증폭할지 결정 (가장 중요)
        #    값이 너무 크면 위치가 폭발하고, 너무 작으면 로봇이 거의 반응하지 않습니다.
        self.sensitivity = 0.02

        # 2. 속도 감쇠: 속도가 무한히 증가하는 것을 막기 위해 계속해서 속도를 줄여주는 힘
        #    1.0에 가까울수록 감쇠가 약하고, 0에 가까울수록 강합니다.
        self.velocity_damping = 0.90

        # 3. 위치 복원: 목표 위치를 계속해서 '홈 위치'로 부드럽게 되돌리려는 힘
        #    1.0에 가까울수록 복원력이 약하고(현재 위치 유지), 0에 가까울수록 강합니다.
        self.position_damping = 0.99

        # =======================================================================

        # 경로 저장을 위한 변수
        self.path_msg = Path()
        self.path_msg.header.frame_id = "world"
        self.max_path_size = 200
        
        self.get_logger().info("Sensor Fusion (Tuned) Node has started.")

    def imu_callback(self, msg):
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return
        
        # 시간 변화량(dt) 계산
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # 1. 현재 자세와 측정된 가속도 가져오기
        orientation_q = msg.orientation
        ros_q = np.array([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        measured_acceleration = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        
        # 쿼터니언 유효성 확인
        if abs(np.linalg.norm(ros_q) - 1.0) > 0.1:
            self.get_logger().warn(f"Received non-normalized quaternion, skipping.", throttle_duration_sec=1.0)
            return
            
        current_rotation = R.from_quat(ros_q)

        # 2. 중력 벡터 계산: 월드 좌표계의 중력(0,0,-9.8)을 현재 IMU 자세를 기준으로 변환
        gravity_vector_world = np.array([0, 0, -9.8])
        gravity_in_imu_frame = current_rotation.inv().apply(gravity_vector_world)

        # 3. 중력 보상: 측정된 가속도에서 중력 성분을 빼서 '순수 이동 가속도'를 얻음
        true_linear_acceleration = measured_acceleration - gravity_in_imu_frame
        
        # 4. 순수 이동 가속도를 월드 좌표계 기준으로 변환
        true_linear_acceleration_world = current_rotation.apply(true_linear_acceleration)

        # 5. 속도 및 위치 업데이트 (안정화 로직 적용)
        # 5a. 속도 감쇠 적용
        self.current_velocity *= self.velocity_damping
        
        # 5b. 계산된 가속도를 속도에 더함 (민감도 적용)
        self.current_velocity += true_linear_acceleration_world * self.sensitivity * dt
        
        # 5c. 계산된 속도를 위치에 더함
        self.current_position += self.current_velocity * dt
        
        # 5d. 위치 복원 적용 (홈 위치로 서서히 돌아가게 만듦)
        home_offset = self.current_position - self.home_position
        self.current_position = self.home_position + (home_offset * self.position_damping)
        
        # 6. 목표 좌표(PoseStamped) 메시지 생성 및 발행
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now.to_msg()
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = self.current_position[0]
        pose_msg.pose.position.y = self.current_position[1]
        pose_msg.pose.position.z = self.current_position[2] # Z축은 현재 제어하지 않지만 함께 업데이트
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
    node = OrientationToPositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
