# arduino_interface/arduino_node.py
# (다이나믹셀 제어 명령 전송 기능이 추가된 최종 브릿지 노드)

import serial
import rclpy
import math
import sys  # sys 모듈 임포트

from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu, MagneticField
# JointTrajectoryPoint 메시지 타입을 임포트
from trajectory_msgs.msg import JointTrajectoryPoint 

class ArduinoInterfaceNode(Node):
    def __init__(self):
        super().__init__('arduino_interface_node')

        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 115200

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Successfully connected to Arduino on {self.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino on {self.serial_port}: {e}")
            # rclpy.shutdown() 대신 sys.exit()를 사용하여 노드 생성 중단
            sys.exit(1)

        # --- Publishers (기존과 동일) ---
        self.feedback_publisher = self.create_publisher(String, 'feedback_from_arduino', 10)
        self.imu_publisher = self.create_publisher(Imu, 'imu/raw', 10)
        self.mag_publisher = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.get_logger().info('Publishing to "/imu/raw" and "/imu/mag" topics.')

        self.timer = self.create_timer(0.01, self.read_from_arduino)
        self.get_logger().info("ArduinoInterfaceNode has started.")

    def read_from_arduino(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if not line:
                    return

                if line.startswith("IMU:"):
                    try:
                        # (이하 IMU 파싱 및 발행 로직은 기존과 동일)
                        imu_data_str = line[len("IMU:"):].split(',')
                        if len(imu_data_str) == 9:
                            accX, accY, accZ, gyroX, gyroY, gyroZ, magX, magY, magZ = map(float, imu_data_str)
                            now = self.get_clock().now().to_msg()

                            # --- Imu 메시지 발행 (가속도, 자이로) ---
                            imu_msg = Imu()
                            imu_msg.header.stamp = now
                            imu_msg.header.frame_id = 'world'
                            imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z = accX, accY, accZ
                            imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z = gyroX, gyroY, gyroZ
                            imu_msg.orientation_covariance[0] = -1.0 # Orientation 정보가 없음을 명시
                            self.imu_publisher.publish(imu_msg)

                            # --- MagneticField 메시지 발행 (지자기) ---
                            mag_msg = MagneticField()
                            mag_msg.header.stamp = now
                            mag_msg.header.frame_id = 'world'
                            mag_msg.magnetic_field.x, mag_msg.magnetic_field.y, mag_msg.magnetic_field.z = magX, magY, magZ
                            self.mag_publisher.publish(mag_msg)
                        else:
                            self.get_logger().warn(f"IMU data format error: {line}")
                    except ValueError as ve:
                        self.get_logger().error(f"Error parsing IMU data: {line} - {ve}")

                else:
                    feedback_msg = String()
                    feedback_msg.data = line
                    self.feedback_publisher.publish(feedback_msg)
                    # self.get_logger().info(f'Received general feedback: "{line}"') # 너무 많은 로그를 방지하기 위해 주석 처리

            except Exception as e:
                self.get_logger().error(f"Error reading from serial: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoInterfaceNode()
    rclpy.spin(node)
    # 노드가 종료될 때 시리얼 포트를 확실히 닫도록 수정
    if hasattr(node, 'ser') and node.ser.is_open:
        node.ser.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
