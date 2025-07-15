import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # ===================================================================================
    # === 1. `arduino_interface` 패키지의 런치 파일 포함 (IMU 센서 노드들 실행) ===
    # ===================================================================================
    # arduino_node.py는 이제 IMU 데이터만 읽어옵니다.
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('arduino_interface'), 'launch', 'start_imu.launch.py'
            ])
        ])
    )

    # =======================================================================
    # === 2. `scara_control` 패키지의 역기구학 노드 실행 ===
    # =======================================================================
    # 이 노드는 이제 ros2_control을 위한 액션(Action)만 보냅니다.
    scara_ik_node = Node(
        package="scara_control",
        executable="scara_ik_node",
        name="scara_ik_node",
        output="screen",
    )

    # ==================================================================
    # === 3. `ros2_control`을 위한 설정 및 노드 실행 (기존과 거의 동일) ===
    # ==================================================================
    # URDF 로드를 위한 설정
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution(
            [FindPackageShare('scara_description'), 'urdf', 'scara_robot.urdf.xacro']
        ),
        ' use_sim:=false' # 실제 로봇용 하드웨어 인터페이스를 로드
    ])
    robot_description = {'robot_description': robot_description_content}

    # 컨트롤러 설정 파일 경로
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("scara_description"),
            "config",
            "scara_controllers.yaml", # 이름이 수정된 버전을 사용
        ]
    )

    # Robot State Publisher 노드
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Controller Manager 노드 (ros2_control의 핵심)
    # 이 노드가 URDF의 dynamixel_hardware_interface를 로드하여 모터를 직접 제어합니다.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen",
    )

    # 컨트롤러 Spawner 노드들
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )
    
    # yaml 파일에서 수정한 이름(joint_trajectory_controller)을 여기서도 사용합니다.
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
    )

    # ======================================================
    # === 4. 위에서 정의한 모든 노드와 런치 파일을 실행 ===
    # ======================================================
    return LaunchDescription([
        imu_launch,
        scara_ik_node,
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
    ])