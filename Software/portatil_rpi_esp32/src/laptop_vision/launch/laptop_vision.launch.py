from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declarar argumentos
    camera_ip_arg = DeclareLaunchArgument(
        'camera_ip',
        default_value='192.168.100.122:5000',
        description='IP:PUERTO del stream de cámara (app IPCamera)'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='roborescue',
        description='Namespace para todos los topics'
    )
    
    target_arg = DeclareLaunchArgument(
        'target',
        default_value='blue_box',
        description='Objetivo al que navegar: blue_box o yellow_box'
    )
    
    domain_id_arg = DeclareLaunchArgument(
        'domain_id',
        default_value='0',
        description='ROS_DOMAIN_ID (usar ID diferente en competencia)'
    )
    
    # Obtener configuraciones de launch
    camera_ip = LaunchConfiguration('camera_ip')
    namespace = LaunchConfiguration('namespace')
    target = LaunchConfiguration('target')
    
    # Nodo 1: Publicador de Cámara
    camera_node = Node(
        package='laptop_vision',
        executable='camera_publisher',
        name='camera_publisher',
        namespace=namespace,  # Namespace a nivel de nodo, no parámetro
        output='screen',
        parameters=[{
            'video_url': ['http://', camera_ip, '/video'],
            'publish_rate': 10.0
        }]
    )
    
    # Nodo 2: Detector ArUco
    detector_node = Node(
        package='laptop_vision',
        executable='aruco_detector',
        name='aruco_detector',
        namespace=namespace,  # Namespace a nivel de nodo, no parámetro
        output='screen',
        parameters=[{
            'robot_id': 1,
            'blue_box_id': 36,
            'yellow_box_id': 47
        }]
    )
    
    # Nodo 3: Navegador ArUco
    navigator_node = Node(
        package='laptop_vision',
        executable='aruco_navigator',
        name='aruco_navigator',
        namespace=namespace,  # Namespace a nivel de nodo, no parámetro
        output='screen',
        parameters=[{
            'target': target,
            'max_linear_speed': 0.6,
            'max_angular_speed': 0.5,
            'linear_p_gain': 1.8,
            'angular_p_gain': 0.3,
            'angular_deadband_deg': 10.0,
            'goal_tolerance': 0.15
        }]
    )
    
    return LaunchDescription([
        camera_ip_arg,
        namespace_arg,
        target_arg,
        domain_id_arg,
        camera_node,
        detector_node,
        navigator_node
    ])
