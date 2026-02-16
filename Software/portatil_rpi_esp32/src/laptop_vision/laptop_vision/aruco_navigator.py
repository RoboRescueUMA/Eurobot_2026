import rclpy
from rclpy.node import Node
import math
import time
from geometry_msgs.msg import Pose2D, Twist

class ArucoNavigator(Node):
    def __init__(self):
        super().__init__('aruco_navigator')
        
        # Declarar parámetros
        self.declare_parameter('target', 'blue_box')  # 'blue_box' o 'yellow_box'
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('linear_p_gain', 0.5)
        self.declare_parameter('angular_p_gain', 1.5)
        self.declare_parameter('angular_deadband_deg', 10.0)  # Zona muerta en grados
        self.declare_parameter('goal_tolerance', 0.1)  # metros
        self.declare_parameter('detection_timeout', 1.0)  # segundos sin detección antes de parar
        
        # Obtener parámetros
        self.objetivo = self.get_parameter('target').get_parameter_value().string_value
        self.vel_lineal_max = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.vel_angular_max = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.ganancia_lineal = self.get_parameter('linear_p_gain').get_parameter_value().double_value
        self.ganancia_angular = self.get_parameter('angular_p_gain').get_parameter_value().double_value
        self.angular_deadband_deg = self.get_parameter('angular_deadband_deg').get_parameter_value().double_value
        self.angular_deadband_rad = math.radians(self.angular_deadband_deg)
        self.tolerancia = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        self.timeout_deteccion = self.get_parameter('detection_timeout').get_parameter_value().double_value
        
        # Suscribirse a posición del objetivo (nombre relativo)
        nombre_topic = f'{self.objetivo}_pose'  # Cambiado de _pos a _pose
        self.subscription = self.create_subscription(
            Pose2D,
            nombre_topic,
            self.callback_posicion,
            10
        )
        
        # Publicar cmd_vel (nombre relativo)
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel_laptop', 10)
        
        # Estado
        self.ultima_pos_objetivo = None
        self.objetivo_alcanzado = False
        self.ultimo_tiempo_deteccion = None
        self.robot_detenido = True  # Empieza detenido
        
        # Timer para verificar timeout de detección (10 Hz)
        self.timer_verificacion = self.create_timer(0.1, self.verificar_timeout)
        
        self.get_logger().info(
            f'✅ Navegador ArUco listo. Objetivo: {self.objetivo}, '
            f'Zona muerta angular: {self.angular_deadband_deg}°, Timeout: {self.timeout_deteccion}s'
        )

    def callback_posicion(self, msg):
        """Calcular cmd_vel basado en posición y orientación del objetivo"""
        try:
            # Actualizar tiempo de última detección
            self.ultimo_tiempo_deteccion = time.time()
            self.robot_detenido = False
            
            # Extraer posición y orientación del objetivo (relativa al robot en marco de cámara)
            objetivo_x = msg.x
            objetivo_y = msg.y
            objetivo_theta = msg.theta  # Orientación relativa: theta_objetivo - theta_robot
            
            # Calcular distancia al objetivo
            distancia = math.sqrt(objetivo_x**2 + objetivo_y**2)
            
            # Verificar si se alcanzó el objetivo
            if distancia < self.tolerancia:
                if not self.objetivo_alcanzado:
                    self.get_logger().info(f'🎯 ¡Objetivo alcanzado! Distancia: {distancia:.3f}m')
                    self.objetivo_alcanzado = True
                    self.enviar_comando_parada()
                    self.robot_detenido = True
                return
            else:
                self.objetivo_alcanzado = False
            
            # ESTRATEGIA SECUENCIAL: 1) Alinearse primero, 2) Moverse en X-Y después
            # Marco de cámara: X=derecha, Y=abajo (coordenadas típicas de imagen)
            # Marco de robot: X=adelante, Y=izquierda
            # Transformar coordenadas de cámara a marco del robot
            # Asumiendo cámara montada cenital mirando hacia abajo:
            #   camara_x -> robot_y (derecha de cámara = derecha del robot)
            #   camara_y -> robot_x (abajo de cámara = adelante del robot)
            marco_robot_x = objetivo_y  # Adelante
            marco_robot_y = -objetivo_x  # Izquierda (negativo porque X de cámara es derecha)
            
            # Control angular usando orientación del ArUco (theta_objetivo - theta_robot)
            # Si objetivo_theta > 0: el objetivo está orientado más a la izquierda que el robot
            # Si objetivo_theta < 0: el objetivo está orientado más a la derecha que el robot
            error_angular = objetivo_theta
            
            # FASE 1: ALINEACIÓN - Primero girar hasta estar alineado
            if abs(error_angular) > self.angular_deadband_rad:
                # Robot desalineado -> SOLO rotar (sin movimiento XY)
                velocidad_angular = self.ganancia_angular * error_angular
                velocidad_angular = max(-self.vel_angular_max, min(self.vel_angular_max, velocidad_angular))
                velocidad_x = 0.0
                velocidad_y = 0.0
                estado = "🔄 ALINEANDO"
            else:
                # FASE 2: MOVIMIENTO - Robot alineado -> Moverse en XY (sin rotar)
                velocidad_angular = 0.0
                
                # Control proporcional en X e Y
                velocidad_x = self.ganancia_lineal * marco_robot_x
                velocidad_y = self.ganancia_lineal * marco_robot_y
                
                # Limitar velocidades lineales
                velocidad_x = max(-self.vel_lineal_max, min(self.vel_lineal_max, velocidad_x))
                velocidad_y = max(-self.vel_lineal_max, min(self.vel_lineal_max, velocidad_y))
                
                estado = "➡️ MOVIENDO"
            
            # Crear y publicar mensaje Twist
            cmd = Twist()
            cmd.linear.x = velocidad_x
            cmd.linear.y = velocidad_y  # Usar strafe lateral de Mecanum
            cmd.linear.z = 0.0
            cmd.angular.x = 0.0
            cmd.angular.y = 0.0
            cmd.angular.z = velocidad_angular
            
            self.pub_cmd_vel.publish(cmd)
            
            # Log de estado con información de orientación
            self.get_logger().info(
                f'🎯 Objetivo: ({objetivo_x:.2f}, {objetivo_y:.2f}), θ={math.degrees(objetivo_theta):.1f}° | '
                f'Distancia: {distancia:.2f}m | Error angular: {math.degrees(error_angular):.1f}° | '
                f'{estado} | Cmd: vx={velocidad_x:.2f}, vy={velocidad_y:.2f}, w={velocidad_angular:.2f}'
            )
            
            self.ultima_pos_objetivo = (objetivo_x, objetivo_y)
            
        except Exception as e:
            self.get_logger().error(f'❌ Error en callback de posición: {e}')
            self.enviar_comando_parada()
            self.robot_detenido = True

    def verificar_timeout(self):
        """Verificar si ha pasado demasiado tiempo sin detectar el objetivo"""
        if self.ultimo_tiempo_deteccion is None:
            # Aún no hemos recibido ninguna detección
            return
        
        tiempo_transcurrido = time.time() - self.ultimo_tiempo_deteccion
        
        # Si pasó el timeout y el robot no está detenido, detenerlo
        if tiempo_transcurrido > self.timeout_deteccion and not self.robot_detenido:
            self.get_logger().warn(
                f'⚠️ Objetivo perdido (sin detección por {tiempo_transcurrido:.1f}s). '
                f'Deteniendo robot...'
            )
            self.enviar_comando_parada()
            self.robot_detenido = True
            self.objetivo_alcanzado = False

    def enviar_comando_parada(self):
        """Enviar comando de velocidad cero"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        self.pub_cmd_vel.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
