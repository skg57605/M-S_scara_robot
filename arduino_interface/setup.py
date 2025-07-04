# ~/ros2_ws/src/arduino_interface/setup.py

import os # os 모듈 임포트
from glob import glob # glob 모듈 임포트
from setuptools import setup

package_name = 'arduino_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ===> 이 부분을 추가해주세요! <===
        # launch 디렉토리와 그 안의 모든 .launch.py 파일을 설치하도록 지정합니다.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seungpyo',
    maintainer_email='skg57605@gmail.com',
    description='Package for IMU interface and processing.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_node = arduino_interface.arduino_node:main',
            'orientation_to_position_node = arduino_interface.orientation_to_position_node:main',
        ],
    },
)
