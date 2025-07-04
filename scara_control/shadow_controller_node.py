# shadow_controller_node.py
# Gazebo의 컨트롤러로 가는 명령을 엿듣고, 실제 로봇에게 전달하는 '그림자' 노드

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
        
        # 실제 로봇(아두이노 브릿지)에게 전달할 토픽을 발행
        self.real_robot_publisher = self.create_publisher(
            JointTrajectoryPoint,
            '/joint_commands', # arduino_serial_bridge.py 가 구독하는 토픽
            10)
        
        self.get_logger().info('Shadow Controller Node has started.')

    def trajectory_callback(self, msg):
        # Gazebo로 가는 궤적(trajectory) 메시지에는 여러 개의 경유점(points)이 있을 수 있음
        # 우리는 그 중 가장 마지막, 즉 최종 목표 지점만 실제 로봇에게 전달하면 됨
        if msg.points:
            final_point = msg.points[-1] # 마지막 포인트를 가져옴
            
            # JointTrajectoryPoint 메시지를 그대로 /joint_commands 토픽으로 발행
            self.real_robot_publisher.publish(final_point)
            
            # 로그 출력 (디버깅용)
            pos_str = ", ".join([f"{p:.3f}" for p in final_point.positions])
            self.get_logger().info(f'Shadowing command: [{pos_str}]', throttle_duration_sec=0.5)

def main(args=None):
    rclpy.init(args=args)
    shadow_node = ShadowControllerNode()
    rclpy.spin(shadow_node)
    shadow_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()