1. 팀원의 ROS 2 작업 공간을 생성하고 src 폴더로 이동합니다.

mkdir -p ~/ros2_ws/src

cd ~/ros2_ws/src

3. 당신이 만든 GitHub 저장소를 복제(clone)합니다.
git clone https://github.com/skg57605/M-S_scara_robot.git

1. 작업 공간의 루트로 이동합니다.
cd ~/ros2_ws

2. rosdep을 사용하여 의존성을 설치합니다.
rosdep install --from-paths src -y --ignore-src

1. 작업 공간 루트에서 빌드합니다.

cd ~/ros2_ws

colcon build

3. 빌드가 완료되면, 터미널에 변경 사항을 적용합니다.
source install/setup.bash

4. 패키지가 잘 설치되었는지 확인합니다.

ros2 pkg list | grep scara

ros2 pkg list | grep arduino

6. 이제 패키지의 런치 파일이나 노드를 실행할 수 있습니다.
ros2 launch arduino_interface start_imu.launch.py

ros2 run scara_control scara_ik_node

ros2 launch scara_description gazebo.launch.py
