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
            'max_linear_speed': 0.25,     # Reducido de 0.4 para movimientos más controlados
            'max_angular_speed': 0.5,     # Giro controlado para buen tracking
            'linear_p_gain': 2.5,         # Aumentado de 1.5 - necesario para generar vy > 0.31 (PWM_MIN=80)
            'angular_p_gain': 0.6,        # Ganancia angular suave
            'angular_deadband_deg': 10.0, # DEPRECATED - usar histéresis en su lugar
            'umbral_histeresis_entrar_deg': 20.0,  # Entrar a modo rotación si |theta| > 20°
            'umbral_histeresis_salir_deg': 10.0,   # Salir de modo rotación si |theta| < 10°
            'goal_tolerance': 0.20,       # Reducido de 0.35m - acercarse más al objetivo
            'detection_timeout': 2.0      # Timeout de detección
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
