# shadow_controller_node.py
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ShadowControllerNode(Node):
    def __init__(self):
        super().__init__('shadow_controller_node')

        # Gazebo 컨트롤러가 발행하는 궤적 메시지를 구독 (엿듣기)
        self.subscription = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self.trajectory_callback,
            10)

        # 실제 로봇(dynamixel-workbench)을 위한 새 컨트롤러 토픽에 발행
        # dynamixel-workbench가 이 토픽을 구독하도록 설정해야 함
        self.real_robot_publisher = self.create_publisher(
            JointTrajectory, # dynamixel-workbench는 JointTrajectory를 받음
            '/dynamixel_workbench/joint_trajectory',
            10)

        self.get_logger().info('Shadow Controller Node has started.')

    def trajectory_callback(self, msg):
        # Gazebo로 가는 궤적 메시지를 그대로 실제 로봇용 토픽으로 다시 발행
        self.real_robot_publisher.publish(msg)
        self.get_logger().info(f'Shadowing trajectory command.', throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    shadow_node = ShadowControllerNode()
    rclpy.spin(shadow_node)
    shadow_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()