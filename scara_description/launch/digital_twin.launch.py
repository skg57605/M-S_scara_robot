import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # 1. Gazebo 시뮬레이션 실행 (use_sim:=true)
    # 컨트롤러 토픽 앞에 'sim' 네임스페이스를 붙여줍니다.
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('scara_gazebo'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'use_sim': 'true'}.items()
    )

    # 2. 실제 로봇 드라이버 실행 (use_sim:=false)
    # 컨트롤러 토픽 앞에 'real' 네임스페이스를 붙여줍니다.
    real_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('scara_description'), 'launch', 'real_robot.launch.py')
        ),
        launch_arguments={'use_sim': 'false'}.items()
    )

    # 3. 제어 명령 분배기 (Topic Multiplexer)
    # scara_arm_controller의 명령을 real과 sim 양쪽으로 복사해서 보내는 노드
    # ros2 topic pub ... /scara_arm_controller/joint_trajectory ... 와 같은 명령을 내리면
    # 이 노드가 받아서 /real/... 와 /sim/... 토픽으로 전달합니다.
    joint_trajectory_multiplexer = Node(
        package='topic_tools',
        executable='relay',
        name='joint_trajectory_multiplexer',
        namespace='scara_arm_controller',
        parameters=[{
            'input_topic': 'joint_trajectory',
            'output_topic': '/real/scara_arm_controller/joint_trajectory'
        }]
    )
    
    # Gazebo용 relay 추가
    joint_trajectory_multiplexer_sim = Node(
        package='topic_tools',
        executable='relay',
        name='joint_trajectory_multiplexer_sim',
        namespace='scara_arm_controller',
        parameters=[{
            'input_topic': 'joint_trajectory',
            'output_topic': '/sim/scara_arm_controller/joint_trajectory'
        }]
    )


    # 4. RViz 실행 (시각화)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(FindPackageShare('scara_description'), 'rviz', 'digital_twin.rviz')]
    )

    return LaunchDescription([
        gazebo_launch,
        real_robot_launch,
        joint_trajectory_multiplexer,
        joint_trajectory_multiplexer_sim,
        rviz_node
    ])