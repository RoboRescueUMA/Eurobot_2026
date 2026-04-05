from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_index_arg = DeclareLaunchArgument(
        "camera_index",
        default_value="-1",
        description="Índice /dev/video para cámara cenital (-1 = autodetectar)",
    )

    return LaunchDescription(
        [

            Node(
                package="robot_localization",
                executable="cerebro_eurobot",
                name="cerebro_eurobot",
                namespace="/roborescue",
                output="screen",
            ),
            Node(
                package="robot_localization",
                executable="controlador_garra",
                name="controlador_garra",
                namespace="/roborescue",
                output="screen",
            ),
        ]
    )
