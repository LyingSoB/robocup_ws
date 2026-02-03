from setuptools import setup
import os
from glob import glob

package_name = 'robot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Required package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # 🔹 CONFIG FILES (YAML + MAP IMAGES)
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shubham',
    maintainer_email='shubham@todo.todo',
    description='Navigation configuration',
    license='TODO',
    entry_points={
        'console_scripts': [],
    },
)

