from setuptools import setup

package_name = 'wheel_command_bridge'

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
    maintainer='shubham',
    maintainer_email='shubham@todo.todo',
    description='Wheel command and encoder serial bridge',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # EXECUTABLE NAME      = python_package.python_file:function
            'wheel_bridge = wheel_command_bridge.bridge_node:main',
        ],
    },
)

