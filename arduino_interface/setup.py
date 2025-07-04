# ~/ros2_ws/src/arduino_interface/setup.py

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seungpyo',
    maintainer_email='seungpyo@todo.todo',
    description='Package for IMU interface and processing.',
    license='Apache 2.0',
    tests_require=['pytest'],
    # ===> 이 부분이 가장 중요합니다! <===
    # 우리가 만든 파이썬 스크립트를 실행 가능한 노드로 등록합니다.
    entry_points={
        'console_scripts': [
            'arduino_node = arduino_interface.arduino_node:main',
            'orientation_to_position_node = arduino_interface.orientation_to_position_node:main',
        ],
    },
)
