from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': '/home/shubham/ros2_ws/src/robot_navigation/maps/arena_map.yaml'
        }]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=['/home/shubham/ros2_ws/src/robot_navigation/config/nav2_params.yaml']
    )

    return LaunchDescription([
        map_server,
        amcl
    ])
