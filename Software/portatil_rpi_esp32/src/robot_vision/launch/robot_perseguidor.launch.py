import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    pkg_robot_vision = get_package_share_directory('robot_vision')

    # 1. DEFINICIÓN DE LA CÁMARA
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_vision, 'launch', 'camera.launch.py')
        )
    )

    # 2. DEFINICIÓN DEL RESET (El "Dedo Virtual")
    # Este proceso ejecuta el comando de reset que hacías a mano
    reset_esp32 = ExecuteProcess(
        cmd=['python3', '-m', 'esptool', '--port', '/dev/ttyUSB0', 'run'],
        name='reset_esp32',
        output='screen'
    )

    # 3. DEFINICIÓN DEL AGENTE (Los Músculos)
    # Lo definimos aquí, pero NO lo lanzamos todavía.
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyUSB0', '-b', '115200'],
        output='screen'
    )

    # 4. DEFINICIÓN DEL CEREBRO (Seguidor ArUco)
    aruco_follower = Node(
        package='robot_vision',
        executable='aruco_follower.py',
        name='aruco_follower',
        output='screen'
    )

    # 5. EL COORDINADOR (Event Handler)
    # Aquí está la magia: Decimos "Cuando 'reset_esp32' termine (OnProcessExit),
    # entonces lanza 'micro_ros_agent'".
    start_agent_after_reset = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=reset_esp32,
            on_exit=[
                LogInfo(msg='¡ESP32 Reseteada! Iniciando Agente Micro-ROS...'),
                micro_ros_agent
            ]
        )
    )

    # 6. LISTA FINAL DE EJECUCIÓN
    return LaunchDescription([
        camera_launch,           # Arranca la cámara
        aruco_follower,          # Arranca el cerebro
        reset_esp32,             # Ejecuta el Reset
        start_agent_after_reset  # Espera a que termine el reset para lanzar el agente
    ])
