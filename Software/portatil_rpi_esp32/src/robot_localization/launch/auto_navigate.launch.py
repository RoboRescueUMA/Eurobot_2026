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
                executable="field_navigator",
                name="field_navigator",
                namespace="/roborescue",
                parameters=[
                    {
                        "target": "yellow_box",
                        "goal_tolerance": 0.20,
                        "max_linear_speed": 0.53,
                        "min_linear_speed": 0.5,
                        "preclear_enabled": True,
                        "preclear_waypoints": "250,77;284,77;278,157",
                        "preclear_tolerance_cm": 10.0,
                        "command_topic": "/roborescue/cmd_vel_laptop",
                    }
                ],
                output="screen",
            ),
        ]
    )
