from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='robot_perception',
            executable='table_plane_detector',
            name='table_plane_detector',
            output='screen'
        ),

        Node(
            package='robot_perception',
            executable='color_object_detector',
            name='color_object_detector',
            output='screen'
        ),

        Node(
            package='robot_perception',
            executable='object_pose_estimator',
            name='object_pose_estimator',
            output='screen'
        ),

        Node(
            package='robot_perception',
            executable='object_pose_transformer',
            name='object_pose_transformer',
            output='screen'
        ),

        Node(
            package='robot_perception',
            executable='manipulation_readiness_gate',
            name='manipulation_readiness_gate',
            output='screen'
        ),

    ])
