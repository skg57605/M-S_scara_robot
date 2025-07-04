# 1. 팀원의 ROS 2 작업 공간을 생성하고 src 폴더로 이동합니다.
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. 당신이 만든 GitHub 저장소를 복제(clone)합니다.
git clone https://github.com/skg57605/M-S_scara_robot.git

# 1. 작업 공간의 루트로 이동합니다.
cd ~/ros2_ws

# 2. rosdep을 사용하여 의존성을 설치합니다.
# 이 명령어는 src 폴더 안의 모든 package.xml을 검사하여
# 필요한 패키지들을 자동으로 설치해 줍니다.
rosdep install --from-paths src -y --ignore-src

# 1. 작업 공간 루트에서 빌드합니다.
cd ~/ros2_ws
colcon build

# 2. 빌드가 완료되면, 터미널에 변경 사항을 적용합니다.
source install/setup.bash

# 3. 패키지가 잘 설치되었는지 확인합니다.
ros2 pkg list | grep scara
ros2 pkg list | grep arduino

# 4. 이제 패키지의 런치 파일이나 노드를 실행할 수 있습니다.
ros2 launch my_awesome_package my_launch_file.launch.py
