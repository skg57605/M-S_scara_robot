# ~/ros2_ws/src/scara_control/scara_control/scara_ik_node.py
# (시뮬레이션과 실제 로봇에 동시에 명령을 보내는 '디지털 트윈' 버전)

import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class ScaraIKNode(Node):
    def __init__(self):
        super().__init__('scara_ik_node')

        # --- 로봇 파라미터 ---
        self.l1 = 0.2
        self.l2 = 0.2
        
        # --- 입력: /target_pose 토픽 구독 ---
        self.subscription = self.create_subscription(
            PoseStamped, '/target_pose', self.target_pose_callback, 10)
            
        # Gazebo와 실제 로봇(dynamixel-workbench) 양쪽에 모두 통용되는 액션 클라이언트
        self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

        self.get_logger().info('IK Node has started.')
        self._action_client.wait_for_server(timeout_sec=5.0)
        self.get_logger().info('Action server (/joint_trajectory_controller) connected.')

    def target_pose_callback(self, msg):
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y

        try:
            theta1, theta2 = self.solve_ik(target_x, target_y)
            self.get_logger().info(f'Target(X:{target_x:.2f}, Y:{target_y:.2f}) -> Angles(T1:{theta1:.2f}, T2:{theta2:.2f})', throttle_duration_sec=0.5)
            
            # 계산된 각도를 두 곳 모두에 전송
            self.send_goal_to_controller(theta1, theta2)

        except ValueError as e:
            self.get_logger().warn(f'IK Error: {e}', throttle_duration_sec=1.0)

    def solve_ik(self, x, y):
        # (이전과 동일한 역기구학 계산 로직)
        l1, l2 = self.l1, self.l2
        d_sq = x**2 + y**2
        max_reach_sq = (l1 + l2)**2
        min_reach_sq = (l1 - l2)**2
        if d_sq > max_reach_sq:
            raise ValueError("Target is outside the reachable workspace(Max).")
        if d_sq < min_reach_sq:
            raise ValueError("Target is outside the reachable workspace(Min).")
            
        cos_theta2 = (d_sq - l1**2 - l2**2) / (2 * l1 * l2)
        sin_theta2 = -math.sqrt(1 - cos_theta2**2)
        theta2 = math.atan2(sin_theta2, cos_theta2)
        k1 = l1 + l2 * math.cos(theta2)
        k2 = l2 * math.sin(theta2)
        theta1 = math.atan2(y, x) - math.atan2(k2, k1)
        return theta1, theta2

    def publish_to_real_robot(self, theta1, theta2):
        # 실제 로봇(아두이노 브릿지)으로 토픽 발행
        point_msg = JointTrajectoryPoint()
        point_msg.positions = [theta1, theta2]
        self.real_robot_publisher.publish(point_msg)

    def send_goal_to_gazebo(self, theta1, theta2):
        # Gazebo로 액션 목표 전송
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['joint_1', 'joint_2']
        
        point = JointTrajectoryPoint()
        point.positions = [theta1, theta2]
        point.time_from_start = Duration(sec=0, nanosec=200000000) # 0.2초
        
        goal_msg.trajectory.points.append(point)
        self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    scara_ik_node = ScaraIKNode()
    rclpy.spin(scara_ik_node)
    scara_ik_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
