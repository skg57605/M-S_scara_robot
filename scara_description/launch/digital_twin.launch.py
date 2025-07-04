# ~/ros2_ws/src/scara_description/launch/digital_twin.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    scara_description_pkg = get_package_share_directory('scara_description')
    arduino_interface_pkg = get_package_share_directory('arduino_interface')

    # 1. Gazebo 시뮬레이션 실행 (시뮬레이션 '몸')
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scara_description_pkg, 'launch', 'gazebo.launch.py')
        )
    )

    # 2. IMU 인터페이스 실행 (센서 입력부)
    start_imu_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arduino_interface_pkg, 'launch', 'start_imu.launch.py')
        )
    )
    
    # 3. 실제 로봇 하드웨어 드라이버 실행 (실제 로봇 '몸')
    start_real_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scara_description_pkg, 'launch', 'real_robot.launch.py')
        )
    )

    # 4. IK 제어 노드 실행 (두뇌)
    start_ik_node = Node(
        package='scara_control',
        executable='scara_ik_node',
        name='scara_ik_node',
        output='screen'
    )

    return LaunchDescription([
        start_gazebo,
        start_imu_interface,
        # start_real_robot, # 실제 로봇을 연결했을 때 이 줄의 주석을 해제하세요!
        start_ik_node
    ])