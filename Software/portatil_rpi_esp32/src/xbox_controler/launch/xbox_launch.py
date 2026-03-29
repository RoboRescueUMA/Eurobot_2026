import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_filepath = os.path.join(
        get_package_share_directory('xbox_controler'),
        'config',
        'teleop_twist_joy.yaml'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'deadzone': 0.1}]
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[config_filepath],
        remappings=[
            ('cmd_vel', '/roborescue/cmd_vel')
        ]
    )

    return LaunchDescription([
        joy_node,
        teleop_node
    ])
