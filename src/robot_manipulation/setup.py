import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_manipulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ROS package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # 🔽 INSTALL CONFIG FILES 🔽
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shubham',
    maintainer_email='shubham@todo.todo',
    description='Robot manipulation logic',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_state_machine = robot_manipulation.arm_state_machine:main',
            'base_yaw_planner = robot_manipulation.base_yaw_planner:main',
            'serial_joint_bridge = robot_manipulation.serial_joint_bridge:main',
            'manipulation_executor = robot_manipulation.manipulation_executor:main',
        ],
    },
)

