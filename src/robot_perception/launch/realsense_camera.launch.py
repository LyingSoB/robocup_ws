from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        output='screen',
        parameters=[{
            'camera_name': 'camera',
            'enable_color': True,
            'enable_depth': True,
            'enable_infra1': True,
            'enable_infra2': True,
            'enable_gyro': False,       # 🔥 critical
            'enable_accel': False,     # 🔥 critical
            'use_v4l2_backend': False, # 🔥 critical
        }],
    )

    return LaunchDescription([realsense_node])

