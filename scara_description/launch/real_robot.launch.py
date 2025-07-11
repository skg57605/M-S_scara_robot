# ~/ros2_ws/src/scara_description/launch/real_robot.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # 1. 'use_sim' 인자를 선언합니다. 실제 로봇을 사용하므로 default_value를 'false'로 설정합니다.
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation (Gazebo) if true'
    )

    # 2. xacro를 실행하여 URDF를 생성합니다. 'use_sim:=false'가 전달됩니다.
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution(
            [FindPackageShare('scara_description'), 'urdf', 'scara_robot.urdf.xacro']
        ),
        ' use_sim:=',
        LaunchConfiguration('use_sim')
    ])
    robot_description = {'robot_description': robot_description_content}

    # 3. 컨트롤러 설정 파일 경로를 가져옵니다.
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("scara_description"),
            "config",
            "scara_controllers.yaml",
        ]
    )

    # 4. ros2_control의 핵심인 Controller Manager 노드를 실행합니다.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen",
    )

    # 5. Robot State Publisher 노드 (로봇의 현재 상태를 TF로 발행)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 6. 컨트롤러들을 로드하고 활성화시키는 Spawner 노드들
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )
    
    load_scara_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['scara_arm_controller', '--controller-manager', '/controller_manager'],
    )

    return LaunchDescription([
        use_sim_arg,
        control_node,
        node_robot_state_publisher,
        load_joint_state_broadcaster,
        load_scara_arm_controller,
    ])