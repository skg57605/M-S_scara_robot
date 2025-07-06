import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    scara_description_pkg = get_package_share_directory('scara_description')
    arduino_interface_pkg = get_package_share_directory('arduino_interface')
    
    # 1. Gazebo 시뮬레이션 환경 실행
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scara_description_pkg, 'launch', 'gazebo.launch.py')
        )
    )

    # 2. 실제 로봇 하드웨어 드라이버 실행
    dynamixel_info_yaml_path = os.path.join(scara_description_pkg, 'config', 'dynamixel_info.yaml')
    scara_controllers_yaml_path = os.path.join(scara_description_pkg, 'config', 'scara_controllers.yaml')
    
    start_real_robot_driver = Node(
        package='dynamixel_workbench_ros',
        executable='dynamixel_workbench_ros',
        name='dynamixel_workbench',
        output='screen',
        parameters=[
            {'use_sim_time': True}, # 디지털 트윈이므로 시뮬레이션 시간에 맞춤
            dynamixel_info_yaml_path,
            scara_controllers_yaml_path
        ]
    )

    # 3. IMU 인터페이스 실행
    start_imu_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arduino_interface_pkg, 'launch', 'start_imu.launch.py')
        )
    )
    
    # 4. IK 제어 노드 실행 ('두뇌')
    start_ik_node = Node(
        package='scara_control',
        executable='scara_ik_node',
        name='scara_ik_node',
        output='screen'
    )

    # 5. 디지털 트윈을 위한 '그림자' 노드 실행
    start_shadow_node = Node(
        package='scara_control',
        executable='shadow_controller_node',
        name='shadow_controller_node',
        output='screen'
    )

    return LaunchDescription([
        start_gazebo,
        start_real_robot_driver,
        start_imu_interface,
        start_ik_node,
        start_shadow_node
    ])
