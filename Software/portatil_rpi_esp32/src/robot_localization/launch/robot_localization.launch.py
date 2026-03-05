from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --- Argumentos ---
    camera_ip_arg = DeclareLaunchArgument(
        'camera_ip',
        default_value='192.168.100.122:5000',
        description='IP:PUERTO del stream de camara (app IPCamera)'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='roborescue',
        description='Namespace ROS2'
    )

    camera_ip = LaunchConfiguration('camera_ip')
    namespace = LaunchConfiguration('namespace')

    # --- Nodo 1: Publicador de camara IP ---
    camera_node = Node(
        package='robot_localization',
        executable='camera_publisher',
        name='zenital_camera_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'video_url': ['http://', camera_ip, '/video'],
            'publish_rate': 10.0,
            'jpeg_quality': 80,
        }]
    )

    # --- Nodo 2: Localizador de campo (homografia + ArUco) ---
    localizer_node = Node(
        package='robot_localization',
        executable='field_localizer',
        name='field_localizer',
        namespace=namespace,
        output='screen',
        parameters=[{
            'robot_id': 1,
            'blue_box_id': 36,
            'yellow_box_id': 47,
            'field_width_cm': 300.0,
            'field_height_cm': 200.0,
            'homography_update_every_n_frames': 30,
        }]
    )

    return LaunchDescription([
        camera_ip_arg,
        namespace_arg,
        camera_node,
        localizer_node,
    ])
