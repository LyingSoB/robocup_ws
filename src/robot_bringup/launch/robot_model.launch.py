from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os

def generate_launch_description():

    use_real_arm = LaunchConfiguration('use_real_arm', default='true')

    robot_description_path = os.path.join(
        get_package_share_directory('robot_description'),
        'urdf',
        'robot.urdf.xacro'
    )

    return LaunchDescription([

        # -------------------------------
        # Robot State Publisher (BASE + ARM)
        # -------------------------------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', robot_description_path]),
                    value_type=str
                )
            }]
        ),

        # -------------------------------
        # Arm joint feedback (potentiometers)
        # -------------------------------
        Node(
            package='robot_manipulation',
            executable='serial_joint_reader',
            name='serial_joint_reader',
            output='screen',
            condition=IfCondition(use_real_arm)
        ),
    ])

