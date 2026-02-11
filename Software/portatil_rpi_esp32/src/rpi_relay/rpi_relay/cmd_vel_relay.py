#!/usr/bin/env python3
"""
Nodo relay simple para reenviar comandos cmd_vel desde laptop al ESP32
Se suscribe a: /robot1/cmd_vel_laptop
Publica a: /cmd_vel (para ESP32 micro-ROS)
Ejecuta en: RPI
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        
        # Declarar parámetros
        self.declare_parameter('namespace', 'robot1')
        
        # Obtener parámetros
        ns = self.get_parameter('namespace').get_parameter_value().string_value
        
        # Suscribirse al topic con namespace desde laptop
        self.subscription = self.create_subscription(
            Twist,
            f'/{ns}/cmd_vel_laptop',
            self.callback_cmd_vel,
            10
        )
        
        # Publicar a topic sin namespace para ESP32 (micro-ROS no soporta namespaces bien)
        self.publicador = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info(f'✅ Nodo relay iniciado. Reenviando /{ns}/cmd_vel_laptop → /cmd_vel')

    def callback_cmd_vel(self, msg):
        """Reenviar el mensaje cmd_vel recibido"""
        self.publicador.publish(msg)
        self.get_logger().debug(f'📡 Reenviando: lineal.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
