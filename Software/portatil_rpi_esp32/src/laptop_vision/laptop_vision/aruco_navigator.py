import rclpy
from rclpy.node import Node
import math
import time
from geometry_msgs.msg import Point, Twist

class ArucoNavigator(Node):
    def __init__(self):
        super().__init__('aruco_navigator')
        
        # Declarar parámetros
        self.declare_parameter('target', 'blue_box')  # 'blue_box' o 'yellow_box'
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('linear_p_gain', 0.5)
        self.declare_parameter('angular_p_gain', 1.5)
        self.declare_parameter('goal_tolerance', 0.1)  # metros
        self.declare_parameter('detection_timeout', 1.0)  # segundos sin detección antes de parar
        
        # Obtener parámetros
        self.objetivo = self.get_parameter('target').get_parameter_value().string_value
        self.vel_lineal_max = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.vel_angular_max = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.ganancia_lineal = self.get_parameter('linear_p_gain').get_parameter_value().double_value
        self.ganancia_angular = self.get_parameter('angular_p_gain').get_parameter_value().double_value
        self.tolerancia = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        self.timeout_deteccion = self.get_parameter('detection_timeout').get_parameter_value().double_value
        
        # Suscribirse a posición del objetivo (nombre relativo)
        nombre_topic = f'{self.objetivo}_pos'
        self.subscription = self.create_subscription(
            Point,
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
        
        self.get_logger().info(f'✅ Navegador ArUco listo. Objetivo: {self.objetivo}, Timeout: {self.timeout_deteccion}s')

    def callback_posicion(self, msg):
        """Calcular cmd_vel basado en posición del objetivo"""
        try:
            # Actualizar tiempo de última detección
            self.ultimo_tiempo_deteccion = time.time()
            self.robot_detenido = False
            
            # Extraer posición del objetivo (relativa al robot en marco de cámara)
            objetivo_x = msg.x
            objetivo_y = msg.y
            
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
            
            # Calcular ángulo al objetivo (atan2 devuelve ángulo en radianes)
            # Marco de cámara: X=derecha, Y=abajo (coordenadas típicas de imagen)
            # Marco de robot: X=adelante, Y=izquierda
            # Necesitamos transformar coordenadas de cámara a marco del robot
            # Asumiendo cámara montada cenital mirando hacia abajo:
            #   camara_x -> robot_y (derecha de cámara = derecha del robot)
            #   camara_y -> robot_x (abajo de cámara = adelante del robot)
            marco_robot_x = objetivo_y  # Adelante
            marco_robot_y = -objetivo_x  # Izquierda (negativo porque X de cámara es derecha)
            
            angulo_al_objetivo = math.atan2(marco_robot_y, marco_robot_x)
            
            # Controlador proporcional simple
            velocidad_lineal = self.ganancia_lineal * distancia
            velocidad_angular = self.ganancia_angular * angulo_al_objetivo
            
            # Limitar velocidades a valores máximos
            velocidad_lineal = max(-self.vel_lineal_max, min(self.vel_lineal_max, velocidad_lineal))
            velocidad_angular = max(-self.vel_angular_max, min(self.vel_angular_max, velocidad_angular))
            
            # Crear y publicar mensaje Twist
            cmd = Twist()
            cmd.linear.x = velocidad_lineal
            cmd.linear.y = 0.0  # Mecanum puede hacer strafe, pero lo mantenemos simple por ahora
            cmd.linear.z = 0.0
            cmd.angular.x = 0.0
            cmd.angular.y = 0.0
            cmd.angular.z = velocidad_angular
            
            self.pub_cmd_vel.publish(cmd)
            
            # Log de estado
            self.get_logger().info(
                f'🎯 Objetivo: ({objetivo_x:.2f}, {objetivo_y:.2f}) | '
                f'Distancia: {distancia:.2f}m | Ángulo: {math.degrees(angulo_al_objetivo):.1f}° | '
                f'Cmd: lineal={velocidad_lineal:.2f}, angular={velocidad_angular:.2f}'
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
