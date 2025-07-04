import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from scipy.spatial.transform import Rotation as R

class IkSolverNode(Node):
    def __init__(self):
        super().__init__('ik_solver_node')
        self.target_pose_sub = self.create_subscription(PoseStamped, '/target_pose', self.target_pose_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.joint_command_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        self.link_lengths = [0.2, 0.2, 0.2, 0.2] 
        self.num_joints = 4 # 제어할 관절은 4개
        self.joint_names = [f"{axis}_{i+1}_joint" for i in range(2) for axis in ["roll", "pitch"]]
        
        self.current_joint_angles = np.zeros(self.num_joints)
        self.target_position = None
        self.get_logger().info("IK Solver Node (4-DOF) has started.")

    def joint_state_callback(self, msg):
        ordered_angles = [0.0] * self.num_joints
        for i, name in enumerate(self.joint_names):
            try:
                idx = msg.name.index(name)
                ordered_angles[i] = msg.position[idx]
            except (ValueError, IndexError): pass
        self.current_joint_angles = np.array(ordered_angles)

    def target_pose_callback(self, msg):
        self.target_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.solve_and_publish_ik()

    def forward_kinematics(self, joint_angles):
        if len(joint_angles) != self.num_joints: return np.identity(4)
        T = np.identity(4)
        # 2개의 능동 세그먼트에 대한 변환 계산
        for i in range(2):
            roll_angle, pitch_angle = joint_angles[i*2], joint_angles[i*2+1]
            origin_z = 0.05/2 if i == 0 else self.link_lengths[i-1]
            T_origin = np.identity(4); T_origin[2,3] = origin_z
            T_roll = np.identity(4); T_roll[:3,:3] = R.from_euler('x', roll_angle).as_matrix()
            T_pitch = np.identity(4); T_pitch[:3,:3] = R.from_euler('y', pitch_angle).as_matrix()
            T = T @ T_origin @ T_roll @ T_pitch

        # 마지막 2개 수동 링크의 변환 추가
        T_link3 = np.identity(4); T_link3[2,3] = self.link_lengths[2]
        T_link4 = np.identity(4); T_link4[2,3] = self.link_lengths[3]
        return T @ T_link3 @ T_link4

    def calculate_jacobian(self, joint_angles):
        J = np.zeros((3, self.num_joints))
        delta = 1e-6
        p_base = self.forward_kinematics(joint_angles)[:3, 3]
        for i in range(self.num_joints):
            angles_perturbed = joint_angles.copy(); angles_perturbed[i] += delta
            p_perturbed = self.forward_kinematics(angles_perturbed)[:3, 3]
            J[:, i] = (p_perturbed - p_base) / delta
        return J

    def solve_and_publish_ik(self):
        if self.target_position is None: return
        q = self.current_joint_angles.copy()
        for _ in range(30):
            current_pos = self.forward_kinematics(q)[:3, 3]
            error = self.target_position - current_pos
            if np.linalg.norm(error) < 0.01: break
            J = self.calculate_jacobian(q)
            delta_q = J.T @ np.linalg.inv(J @ J.T + 0.01 * np.identity(3)) @ error
            delta_q_norm = np.linalg.norm(delta_q)
            if delta_q_norm > 0.1: delta_q = delta_q / delta_q_norm * 0.1
            q += 0.1 * delta_q
        self.publish_joint_command(q)

    def publish_joint_command(self, joint_angles):
        traj_msg = JointTrajectory(); traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.joint_names = self.joint_names
        point = JointTrajectoryPoint(); point.positions = list(joint_angles)
        point.time_from_start.nanosec = 200000000
        traj_msg.points.append(point)
        self.joint_command_pub.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args); node = IkSolverNode()
    rclpy.spin(node); node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()

