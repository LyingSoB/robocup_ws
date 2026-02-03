from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shubham',
    maintainer_email='shubham@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'object_pose_estimator = robot_perception.object_pose_estimator:main',
            'table_plane_detector = robot_perception.table_plane_detector:main',
            'color_object_detector = robot_perception.object_detector_color:main',
            'object_pose_transformer = robot_perception.object_pose_transformer:main',
            'manipulation_readiness_gate = robot_perception.manipulation_readiness_gate:main',
        ],
    },
)
