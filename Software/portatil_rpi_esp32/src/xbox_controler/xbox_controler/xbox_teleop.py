import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class XboxTeleop(Node):
    def __init__(self):
        super().__init__('xbox_teleop')

        # Parámetros configurables
        self.declare_parameter('axis_linear_x', 1)    # Stick izquierdo arriba/abajo
        self.declare_parameter('axis_linear_y', 0)    # Stick izquierdo izq/der (strafe)
        self.declare_parameter('axis_angular_yaw', 3) # Stick derecho izq/der
        self.declare_parameter('enable_button', 5)    # Botón RB
        self.declare_parameter('scale_linear_x', 1.0)
        self.declare_parameter('scale_linear_y', -1.0)
        self.declare_parameter('scale_angular_yaw', -1.0)
        self.declare_parameter('cmd_vel_topic', '/roborescue/cmd_vel')

        self.axis_lx = self.get_parameter('axis_linear_x').get_parameter_value().integer_value
        self.axis_ly = self.get_parameter('axis_linear_y').get_parameter_value().integer_value
        self.axis_yaw = self.get_parameter('axis_angular_yaw').get_parameter_value().integer_value
        self.enable_btn = self.get_parameter('enable_button').get_parameter_value().integer_value
        self.scale_lx = self.get_parameter('scale_linear_x').get_parameter_value().double_value
        self.scale_ly = self.get_parameter('scale_linear_y').get_parameter_value().double_value
        self.scale_yaw = self.get_parameter('scale_angular_yaw').get_parameter_value().double_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value

        self.sub_joy = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.get_logger().info(
            f'Xbox teleop listo. Publicando en {cmd_vel_topic}. '
            f'Mantén botón {self.enable_btn} (RB) pulsado para mover el robot.'
        )

    def joy_callback(self, msg: Joy):
        # Verificar botón de seguridad
        if len(msg.buttons) <= self.enable_btn or msg.buttons[self.enable_btn] == 0:
            return

        cmd = Twist()

        if len(msg.axes) > self.axis_lx:
            cmd.linear.x = self.scale_lx * msg.axes[self.axis_lx]
        if len(msg.axes) > self.axis_ly:
            cmd.linear.y = self.scale_ly * msg.axes[self.axis_ly]
        if len(msg.axes) > self.axis_yaw:
            cmd.angular.z = self.scale_yaw * msg.axes[self.axis_yaw]

        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = XboxTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
