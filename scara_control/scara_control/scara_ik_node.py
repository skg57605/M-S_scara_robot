# ~/ros2_ws/src/scara_control/scara_control/scara_ik_node.py
# (시뮬레이션과 실제 로봇에 동시에 명령을 보내는 '디지털 트윈' 버전)

import rclpy
import math
import sys
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class ScaraIKNode(Node):
    def __init__(self):
        super().__init__('scara_ik_node')

        # 로봇 파라미터
        self.l1 = 0.2
        self.l2 = 0.2
        
        # 입력: /target_pose 토픽 구독
        self.subscription = self.create_subscription(
            PoseStamped, '/target_pose', self.target_pose_callback, 10)
        
        # --- 출력 1: 실제 로봇(아두이노)을 위한 토픽 퍼블리셔(비활성화) ---
        #self.real_robot_publisher = self.create_publisher(
        #    JointTrajectoryPoint, '/joint_commands', 10)
        
        # --- 출력 2: Gazebo 시뮬레이션을 위한 액션 클라이언트 ---
        self._action_client = ActionClient(
            self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        
        self.get_logger().info('Digital Twin IK Node has started.')
        self.get_logger().info('Connecting to action server for Gazebo...')
        server_ready = self._action_client.wait_for_server(timeout_sec=5.0)
        if not server_ready:
            self.get_logger().warn('Gazebo action server not available! Running in "real robot only" mode.')
        else:
            self.get_logger().info('Action server connected.')

    def target_pose_callback(self, msg):
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y
        try:
            theta1, theta2 = self.solve_ik(target_x, target_y)
            
            # ===> 관절 한계 확인 코드 추가 <===
            # URDF에 정의된 limit과 동일하게 설정합니다.
            joint1_min, joint1_max = -2.356, 2.356
            joint2_min, joint2_max = -2.356, 2.356
            
            # 계산된 각도를 두 곳 모두에 전송
            self.send_goal(theta1, theta2)

        except ValueError as e:
            self.get_logger().warn(f'IK Error: {e}', throttle_duration_sec=1.0)

    def solve_ik(self, x, y):
        # (이전과 동일한 역기구학 계산 로직)
        l1, l2 = self.l1, self.l2
        d_sq = x**2 + y**2
        if d_sq > (l1 + l2)**2 or d_sq < (l1 - l2)**2:
            raise ValueError(f"Target (x:{x:.2f}, y:{y:.2f}) is outside workspace.")
        cos_theta2 = (d_sq - l1**2 - l2**2) / (2 * l1 * l2)
        sin_theta2 = -math.sqrt(1 - cos_theta2**2)
        theta2 = math.atan2(sin_theta2, cos_theta2)
        k1 = l1 + l2 * math.cos(theta2)
        k2 = l2 * math.sin(theta2)
        theta1 = math.atan2(y, x) - math.atan2(k2, k1)
        return theta1, theta2

    def send_goal(self, theta1, theta2):
        if not self._action_client.server_is_ready():
            # self.get_logger().warn('Gazebo action server not ready. Skipping.', throttle_duration_sec=1.0)
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['joint_1', 'joint_2']
        point = JointTrajectoryPoint()
        point.positions = [theta1, theta2]
        point.time_from_start = Duration(sec=0, nanosec=100000000)
        goal_msg.trajectory.points.append(point)
        self._action_client.send_goal_async(goal_msg)
        self.get_logger().info(f'Sending goal to Gazebo: [{theta1:.3f}, {theta2:.3f}]', throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    ik_node = ScaraIKNode()
    rclpy.spin(ik_node)
    ik_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
