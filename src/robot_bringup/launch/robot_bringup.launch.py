from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # -----------------------------
    # Paths
    # -----------------------------
    nav2_launch = os.path.join(
        get_package_share_directory('robot_bringup'),
        'launch',
        'nav2.launch.py'
    )

    # -----------------------------
    # Base (dummy / real)
    # -----------------------------
    base_node = Node(
        package='robot_base',
        executable='mecanum_base',
        name='base_controller',
        output='screen'
    )

    # -----------------------------
    # Odometry (simulated for now)
    # -----------------------------
    odom_node = Node(
        package='robot_odometry',
        executable='simulated_odometry',
        name='odometry',
        output='screen'
    )

    # -----------------------------
    # TEMP static TF (REMOVE on real robot)
    # -----------------------------
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen'
    )

    # -----------------------------
    # Nav2
    # -----------------------------
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'autostart': 'true'
        }.items()
    )

    # -----------------------------
    # Task Manager
    # -----------------------------
    task_manager = Node(
        package='task_manager',
        executable='task_manager_node',
        name='task_manager',
        output='screen'
    )

    return LaunchDescription([
        base_node,
        odom_node,
        static_tf,     # ❌ REMOVE later
        nav2,
        task_manager
    ])
