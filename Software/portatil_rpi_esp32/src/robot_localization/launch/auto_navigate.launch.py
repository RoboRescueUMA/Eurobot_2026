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
            camera_index_arg,
            Node(
                package="robot_localization",
                executable="field_localizer",
                name="field_localizer",
                namespace="/roborescue",
                parameters=[
                    {
                        "camera_index": LaunchConfiguration("camera_index"),
                        "modo_simulacion": False,
                        "robot_id": 1,
                        "robot_marker_height_cm": 37.0,
                        "box_marker_height_cm": 3.0,
                    }
                ],
                output="screen",
            ),
            Node(
                package="robot_localization",
                executable="cerebro_eurobot",
                name="cerebro_eurobot",
                namespace="/roborescue",
                output="screen",
            ),
        ]
    )
