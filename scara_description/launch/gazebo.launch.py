# ~/ros2_ws/src/scara_description/launch/gazebo.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # 1. 'use_sim' 인자를 선언하고, launch 시에 값을 받을 수 있도록 준비합니다.
    #    (이 launch 파일을 실행할 때 'use_sim:=false' 같은 인자를 줄 수 있습니다)
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Use simulation (Gazebo) if true'
    )

    # 2. xacro 명령을 실행하여 URDF를 생성하는 부분을 정의합니다.
    #    'Command'를 사용하면 launch가 실행되는 시점에 xacro가 실행되어,
    #    'use_sim' 인자 값을 올바르게 전달할 수 있습니다.
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution(
            [FindPackageShare('scara_description'), 'urdf', 'scara_robot.urdf.xacro']
        ),
        # 여기에 ' use_sim:=true' 라는 텍스트를 xacro 명령에 추가합니다.
        ' use_sim:=',
        LaunchConfiguration('use_sim')
    ])
    robot_description = {'robot_description': robot_description_content}


    # 3. Robot State Publisher 노드
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 4. Gazebo 실행
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        # Gazebo가 시뮬레이션 시간을 사용하도록 설정합니다.
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 5. URDF 모델을 Gazebo에 스폰(Spawn)하는 노드
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'scara_robot'],
        output='screen'
    )
    
    # 6. 컨트롤러 로더 (이름을 컨트롤러 yaml 파일과 일치시킵니다)
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )
    
    load_scara_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        # YAML 파일에 정의된 컨트롤러 이름('scara_arm_controller')과 일치시키는 것이 좋습니다.
        arguments=['scara_arm_controller', '--controller-manager', '/controller_manager'],
    )

    return LaunchDescription([
        use_sim_arg,  # 선언한 인자를 launch description에 추가
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        load_joint_state_broadcaster,
        load_scara_arm_controller,
    ])