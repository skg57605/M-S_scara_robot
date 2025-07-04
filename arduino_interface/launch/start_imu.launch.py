# arduino_interface/launch/start_imu.launch.py (최종 완성본)

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 아두이노 인터페이스 노드
        Node(
            package='arduino_interface',
            executable='arduino_node',
            name='arduino_interface_node',
            output='screen'
        ),
        
        # 2. IMU 필터 노드
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_node',
            output='screen',
            parameters=[{'use_mag': True, 'fixed_frame': 'odom'
            }],
    # arduino_node의 /imu/raw 출력을 필터의 /imu/data_raw 입력으로 연결
            remappings=[('/imu/data_raw', '/imu/raw')]
        ),
        
        # 3. 자세 -> 위치 변환 노드
        Node(
            package='arduino_interface',
            executable='orientation_to_position_node',
            name='orientation_to_position_node',
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_odom_static_publisher',
            arguments=[
            '0',
            '0',
            '0',
            '-1.5707963',
            '0',
            '0',
            'world',
            'odom']
        )
    ])
