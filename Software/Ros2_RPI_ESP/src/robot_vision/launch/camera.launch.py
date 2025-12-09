from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    camera_node = Node(
            package='camera_ros',
            executable='camera_node',
            name='camera_node',
            parameters=[
                {'use_image_transport': True}, # Compresión JPEG para que vaya rápido
                {'camera_name': 'default_camera'},
                {'image_width': 640},
		{'width':640},			
                {'height': 480},		
                {'framerate': 15.0},

                # EL TRUCO: Forzar el formato que RQT entiende
                {'format': 'BGR888'}, 
            ],
            output='screen'
        )

    return LaunchDescription([
        camera_node
    ])
