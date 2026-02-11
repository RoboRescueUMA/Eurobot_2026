#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArucoFollower(Node):
    def __init__(self):
        super().__init__('aruco_follower')

        # Suscripción a la cámara
        self.subscription = self.create_subscription(
            Image,
            '/camera_node/image_raw',
            self.image_callback,
            10)
        
        # Publicar órdenes de movimiento
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.br = CvBridge()

        # Configuración ArUco (Compatible con versiones viejas y nuevas)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        try:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters()

        # --- PARÁMETROS DE CONTROL ---
        self.image_center_x = 160   # Mitad de 320
        self.kp_angular = 0.004     # Sensibilidad de giro
        self.speed_forward = 0.12   # Velocidad de avance
        
        # ZONA MUERTA (HISTÉRESIS)
        self.stop_area = 15000      # Muy cerca -> PARAR
        self.resume_area = 13500    # Se alejó lo suficiente -> VOLVER A EMPEZAR

        self.get_logger().info('¡PERSEGUIDOR v3 (FRENO TOTAL) INICIADO!')

    def image_callback(self, msg):
        cmd = Twist()
        
        try:
            current_frame = self.br.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return

        gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            # Cogemos el primer marcador detectado
            c = corners[0][0]
            
            # Calcular centro X y Área
            center_x = int((c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4)
            area = cv2.contourArea(c)

            # Calculamos cuánto tendríamos que girar SI nos estuviéramos moviendo
            error_x = self.image_center_x - center_x
            proposed_angular = float(error_x * self.kp_angular)

            # --- MÁQUINA DE ESTADOS ---

            if area >= self.stop_area:
                # CASO A: LLEGAMOS A LA META (STOP)
                # Aquí forzamos TODO a cero. 
                # Si no ponemos angular.z = 0, seguiría intentando corregir el giro.
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0  
                self.get_logger().info(f'STOP TOTAL (Llegada) - Área: {int(area)}')
            
            elif area < self.resume_area:
                # CASO B: ESTAMOS LEJOS (AVANZAR)
                # Aquí sí aplicamos velocidad y el giro calculado
                cmd.linear.x = self.speed_forward
                cmd.angular.z = proposed_angular
                self.get_logger().info(f'AVANZANDO - Área: {int(area)}')
                
            else:
                # CASO C: ZONA MUERTA (PAZ)
                # Estamos entre 13500 y 15000. Nos quedamos quietos para no rebotar.
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.get_logger().info(f'EN ZONA DE PAZ - Área: {int(area)}')

        else:
            # Si no ve ningún código -> PARADA DE SEGURIDAD
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # Enviar la orden final
        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
