# ~/ros2_ws/src/scara_description/launch/gazebo.launch.py
# (시뮬레이션 전용으로 정리된 최종 버전)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # 패키지 경로 설정
    pkg_path = get_package_share_directory('scara_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'scara_robot.urdf.xacro')
    
    # XACRO 파일을 로드하고 URDF로 변환
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Robot State Publisher 노드
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}] # 시뮬레이션 시간 사용
    )

    # Gazebo 실행
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ])
    )

    # URDF 모델을 Gazebo에 스폰(Spawn)하는 노드
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'scara_robot'],
        output='screen'
    )

    # 시뮬레이션용 ros2_control 컨트롤러를 로드하는 spawner 노드
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )
    
    load_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        load_joint_state_broadcaster,
        load_joint_trajectory_controller
    ])
