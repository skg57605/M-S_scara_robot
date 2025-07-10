from setuptools import find_packages, setup

package_name = 'scara_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seungpyo',
    maintainer_email='skg57605@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'scara_ik_node = scara_control.scara_ik_node:main',
        'shadow_controller_node = scara_control.shadow_controller_node:main',
        ],
    },
)
