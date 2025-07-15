# arduino_interface/launch/start_tracker.launch.py
# (IMU 위치 추적(position_tracker) 노드를 실행하는 런치 파일)

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    이 런치 파일은 다음 노드들을 순서대로 실행하여 IMU의 물리적 위치를 추적합니다.
    1. arduino_node: 아두이노에서 raw IMU 데이터를 읽어와 /imu/raw 토픽으로 발행합니다.
    2. imu_filter_node: /imu/raw 데이터를 받아 필터링하고 자세(orientation)를 계산하여 /imu/data 토픽으로 발행합니다.
    3. position_tracker_node: 필터링된 /imu/data를 구독하여 바이어스 보정 및 이중 적분을 통해 실제 위치를 추적하고 /tracked_pose로 발행합니다.
    """
    return LaunchDescription([
        # 1. 아두이노 인터페이스 노드 (하드웨어 데이터 수신)
        Node(
            package='arduino_interface',
            executable='arduino_node',
            name='arduino_interface_node',
            output='screen'
        ),
        
        # 2. IMU 필터 노드 (자세 계산)
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_node',
            output='screen',
            parameters=[{'use_mag': True, 
                         'fixed_frame': 'odom'}],
            # arduino_node의 출력(/imu/raw)을 필터 노드의 입력(/imu/data_raw)으로 연결
            remappings=[('/imu/data_raw', '/imu/raw')]
        ),
        
        # =================================================================
        # ===> 3. 자세 -> 위치 변환 노드를 '위치 추적 노드'로 교체 <===
        # =================================================================
        Node(
            package='arduino_interface',
            executable='position_tracker', # 실행 파일 이름 변경
            name='position_tracker_node',   # 노드 이름 변경
            output='screen'
        ),
        # =================================================================

        # 4. world와 odom 좌표계 간의 정적 변환 관계 설정
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_odom_static_publisher',
            # x, y, z, roll, pitch, yaw 순서로 world -> odom 변환
            # 여기서는 -90도(roll) 회전 변환을 적용
            arguments=['0', '0', '0', '-1.5707963', '0', '0', 'world', 'odom']
        )
    ])