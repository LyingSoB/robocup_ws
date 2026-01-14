from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # -----------------------------
    # Nav2 launch path
    # -----------------------------
    nav2_launch = os.path.join(
        get_package_share_directory('robot_bringup'),
        'launch',
        'nav2.launch.py'
    )

    # -----------------------------
    # Wheel encoder bridge (Arduino)
    # MUST start first
    # -----------------------------
    wheel_bridge = Node(
        package='wheel_command_bridge',
        executable='wheel_bridge',
        name='wheel_bridge',
        output='screen'
    )

    # -----------------------------
    # Encoder-based odometry (REAL)
    # Publishes /odom + TF odom→base_link
    # -----------------------------
    odom_node = Node(
        package='robot_odometry',
        executable='encoder_odometry',
        name='encoder_odometry',
        output='screen'
    )

    # -----------------------------
    # Base controller (no motors yet)
    # -----------------------------
    base_node = Node(
        package='robot_base',
        executable='mecanum_base',
        name='base_controller',
        output='screen'
    )

    # -----------------------------
    # Nav2 stack
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
        wheel_bridge,
        odom_node,
        base_node,
        nav2,
        task_manager
    ])

