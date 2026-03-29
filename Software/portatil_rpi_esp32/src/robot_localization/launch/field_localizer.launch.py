from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_index_arg = DeclareLaunchArgument(
        'camera_index', default_value='-1',
        description='Índice /dev/video para la cámara cenital (-1 = autodetectar)'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='/roborescue',
        description='Namespace ROS para el localizador'
    )

    nodo_field_localizer = Node(
        package='robot_localization',
        executable='field_localizer',
        name='field_localizer',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'modo_simulacion': False,
            'robot_id': 1,
            'robot_marker_height_cm': 23.0,
            'camera_index': LaunchConfiguration('camera_index'),
        }],
        output='screen'
    )

    return LaunchDescription([
        camera_index_arg,
        namespace_arg,
        nodo_field_localizer,
    ])
