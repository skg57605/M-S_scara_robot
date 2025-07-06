# ~/ros2_ws/src/scara_description/launch/real_robot.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 설정 파일들의 경로를 scara_description 패키지 기준으로 지정
    scara_description_pkg = get_package_share_directory('scara_description')
    dynamixel_info_yaml_path = os.path.join(scara_description_pkg, 'config', 'dynamixel_info.yaml')
    scara_controllers_yaml_path = os.path.join(scara_description_pkg, 'config', 'scara_controllers.yaml')
    
    # 다이나믹셀 워크벤치 노드 (하드웨어 드라이버) 실행
    dynamixel_workbench_node = Node(
        package='dynamixel_workbench_ros',
        executable='dynamixel_workbench_ros',
        name='dynamixel_workbench',
        output='screen',
        parameters=[
            {'use_sim_time': False}, # 실제 로봇이므로 시뮬레이션 시간을 사용하지 않음
            dynamixel_info_yaml_path,
            scara_controllers_yaml_path
        ]
    )

    return LaunchDescription([
        dynamixel_workbench_node
    ])