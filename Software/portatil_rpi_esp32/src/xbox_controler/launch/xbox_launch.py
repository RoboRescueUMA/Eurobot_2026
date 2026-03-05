import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_filepath = os.path.join(
        get_package_share_directory('xbox_controler'),
        'config',
        'xbox.yaml'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'deadzone': 0.1}]
    )

    xbox_teleop_node = Node(
        package='xbox_controler',
        executable='xbox_teleop',
        name='xbox_teleop',
        parameters=[config_filepath]
    )

    return LaunchDescription([
        joy_node,
        xbox_teleop_node
    ])
