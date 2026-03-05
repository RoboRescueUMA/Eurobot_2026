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
        self.declare_parameter('umbral_histeresis_entrar_deg', 20.0)  # Entrar a modo rotación
        self.declare_parameter('umbral_histeresis_salir_deg', 10.0)   # Salir de modo rotación
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
        
        # Parámetros de histéresis
        self.umbral_histeresis_entrar_deg = self.get_parameter('umbral_histeresis_entrar_deg').get_parameter_value().double_value
        self.umbral_histeresis_salir_deg = self.get_parameter('umbral_histeresis_salir_deg').get_parameter_value().double_value
        self.umbral_histeresis_entrar_rad = math.radians(self.umbral_histeresis_entrar_deg)
        self.umbral_histeresis_salir_rad = math.radians(self.umbral_histeresis_salir_deg)
        
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
        self.en_modo_rotacion = False  # Estado de histéresis para rotación
        
        # Timer para verificar timeout de detección (10 Hz)
        self.timer_verificacion = self.create_timer(0.1, self.verificar_timeout)
        
        self.get_logger().info(
            f'✅ Navegador ArUco listo. Objetivo: {self.objetivo}, '
            f'Histéresis: entrar>{self.umbral_histeresis_entrar_deg}° / salir<{self.umbral_histeresis_salir_deg}°, '
            f'Tolerancia objetivo: {self.tolerancia}m, Timeout: {self.timeout_deteccion}s'
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
            
            # DEBUG: Siempre mostrar comparación de distancia
            self.get_logger().info(
                f'🔍 DEBUG: distancia={distancia:.3f}m, tolerancia={self.tolerancia:.3f}m, '
                f'¿distancia < tolerancia? {distancia < self.tolerancia}'
            )
            
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
            
            # Las coordenadas (objetivo_x, objetivo_y) ya vienen en el frame del robot
            # gracias a la transformación de rotación en aruco_detector.py
            # 
            # Marco de robot (estándar ROS): X=adelante, Y=izquierda
            marco_robot_x = objetivo_x
            marco_robot_y = objetivo_y  # SIN inversión, el detector ya da coordenadas correctas
            
            # Usar orientación relativa (Theta) entre las cajas para decidir si girar
            # objetivo_theta = orientación de la caja objetivo - orientación del robot
            # Si theta > 0: caja rotada a la izquierda respecto al robot
            # Si theta < 0: caja rotada a la derecha respecto al robot
            # Si theta ≈ 0: cajas alineadas -> moverse en diagonal con omniruedas
            angulo_orientacion = objetivo_theta
            
            # HISTÉRESIS: Prevenir oscilación entre modos GIRANDO y AVANZANDO
            # - Si NO estamos rotando: entrar a rotación si |theta| > umbral_entrar
            # - Si YA estamos rotando: salir de rotación si |theta| < umbral_salir
            abs_theta = abs(angulo_orientacion)
            
            if not self.en_modo_rotacion:
                # Actualmente en modo avance -> verificar si debemos entrar a rotación
                if abs_theta > self.umbral_histeresis_entrar_rad:
                    self.en_modo_rotacion = True
            else:
                # Actualmente en modo rotación -> verificar si debemos salir a avance
                if abs_theta < self.umbral_histeresis_salir_rad:
                    self.en_modo_rotacion = False
            
            # FASE 1: ALINEACIÓN - Girar SOLO sobre sí mismo (sin moverse)
            # Giro MUY LENTO para que cámara mantenga tracking
            if self.en_modo_rotacion:
                # Robot desalineado -> SOLO girar, SIN movimiento XY
                velocidad_angular = self.ganancia_angular * angulo_orientacion
                velocidad_angular = max(-self.vel_angular_max, min(self.vel_angular_max, velocidad_angular))
                velocidad_x = 0.0
                velocidad_y = 0.0
                estado = "🔄 GIRANDO EN EL SITIO"
            else:
                # FASE 2: AVANCE - Robot alineado -> Moverse hacia la caja
                velocidad_angular = 0.0
                
                # Movimiento XY hacia la caja (sin rotar más)
                velocidad_x = self.ganancia_lineal * marco_robot_x
                velocidad_y = self.ganancia_lineal * marco_robot_y
                
                # REDUCIR VELOCIDAD gradualmente cuando se acerca (para mejor control)
                # Zona de desaceleración: desde 0.50m hasta goal_tolerance
                ZONA_DESACELERACION = 0.50  # Empezar a reducir desde 50cm
                if distancia < ZONA_DESACELERACION:
                    factor_reduccion = distancia / ZONA_DESACELERACION  # De 1.0 a 0.0
                    factor_reduccion = max(0.3, factor_reduccion)  # Mínimo 30% de velocidad
                    velocidad_x *= factor_reduccion
                    velocidad_y *= factor_reduccion
                
                # El ESP32 maneja PWM_MIN=80 automáticamente
                # No forzamos velocidad mínima aquí
                
                # Limitar velocidades lineales
                velocidad_x = max(-self.vel_lineal_max, min(self.vel_lineal_max, velocidad_x))
                velocidad_y = max(-self.vel_lineal_max, min(self.vel_lineal_max, velocidad_y))
                
                estado = "➡️ AVANZANDO"
            
            # Crear y publicar mensaje Twist
            cmd = Twist()
            # Sin inversiones - la transformación en aruco_detector ya maneja las coordenadas correctamente
            cmd.linear.x = velocidad_x
            cmd.linear.y = velocidad_y
            cmd.linear.z = 0.0
            cmd.angular.x = 0.0
            cmd.angular.y = 0.0
            cmd.angular.z = velocidad_angular
            
            self.pub_cmd_vel.publish(cmd)
            
            # Log de estado
            self.get_logger().info(
                f'🎯 Objetivo: ({objetivo_x:.2f}, {objetivo_y:.2f}) | '
                f'Marco robot: X={marco_robot_x:.2f}m, Y={marco_robot_y:.2f}m | '
                f'Dist: {distancia:.2f}m | Theta: {math.degrees(angulo_orientacion):.1f}° | '
                f'ModoRot: {self.en_modo_rotacion} | '
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
