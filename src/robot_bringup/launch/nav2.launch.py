from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    robot_navigation_dir = get_package_share_directory('robot_navigation')

    nav2_params = os.path.join(
        robot_navigation_dir,
        'config',
        'nav2_params.yaml'
    )

    map_yaml = os.path.join(
        robot_navigation_dir,
        'config',
        'arena_map.yaml'
    )

    return LaunchDescription([

        # -------------------------
        # MAP SERVER (STATIC MAP)
        # -------------------------
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'yaml_filename': map_yaml},
                {'use_sim_time': False}
            ]
        ),

        # -------------------------
        # NAV2 CORE SERVERS
        # -------------------------
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[nav2_params]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            output='screen',
            parameters=[nav2_params]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            output='screen',
            parameters=[nav2_params]
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            output='screen',
            parameters=[nav2_params]
        ),

        # -------------------------
        # LIFECYCLE MANAGER
        # -------------------------
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'controller_server',
                    'planner_server',
                    'bt_navigator',
                    'behavior_server'
                ]
            }]
        ),
    ])

