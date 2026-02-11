import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

# --- CONFIGURACIÓN ---
TAMAÑO_ARUCO = 0.05  
MI_ROBOT_ID = 1
ID_OBJETIVO = [36]   # Cajas llenas
KP_LIN = 1.5         # Ganancia Lineal
DISTANCIA_STOP = 0.15 # 15cm

class EurobotController(Node):
    def __init__(self):
        super().__init__('eurobot_controller')
        self.bridge = CvBridge()
        
        self.sub = self.create_subscription(Image, '/zenital/image_raw', self.image_callback, 1)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.debug_pub = self.create_publisher(Image, '/zenital/debug', 1)

        # Configuración ArUco con COMPATIBILIDAD DE VERSIONES
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        
        try:
            # Intento para OpenCV moderno (4.7+)
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.use_new_api = True
            self.get_logger().info('✅ Usando OpenCV API Nueva (ArucoDetector)')
        except AttributeError:
            # Fallback para OpenCV antiguo (<4.6)
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.use_new_api = False
            self.get_logger().info('⚠️ Usando OpenCV API Antigua (detectMarkers)')

    def get_angle(self, corners):
        """Calcula Yaw en radianes"""
        c = corners[0]
        dx = c[1][0] - c[0][0]
        dy = c[1][1] - c[0][1]
        return math.atan2(dy, dx)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = np.ascontiguousarray(frame)
            
            # --- DETECCIÓN SEGÚN VERSIÓN ---
            if self.use_new_api:
                corners, ids, _ = self.detector.detectMarkers(frame)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)
            
            robot_pos = None
            robot_yaw = 0.0
            target_pos = None
            px_per_m = 0

            if ids is not None:
                ids = ids.flatten()
                
                # Dibujar marcadores detectados
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                for i, marker_id in enumerate(ids):
                    # Centro del marcador
                    c = corners[i][0]
                    cX = int(np.mean(c[:, 0]))
                    cY = int(np.mean(c[:, 1]))

                    if marker_id == MI_ROBOT_ID:
                        robot_pos = np.array([cX, cY])
                        # Escala
                        perimetro = cv2.arcLength(corners[i], True)
                        px_per_m = perimetro / (TAMAÑO_ARUCO * 4)
                        # Ángulo
                        robot_yaw = self.get_angle(corners[i])
                        
                        # Flecha visual del robot
                        end_pt = (int(cX + 50 * math.cos(robot_yaw)), int(cY + 50 * math.sin(robot_yaw)))
                        cv2.arrowedLine(frame, (cX, cY), end_pt, (0,0,255), 2)

                    elif marker_id in ID_OBJETIVO:
                        target_pos = np.array([cX, cY])
                        cv2.circle(frame, (cX, cY), 5, (255, 0, 0), -1)

                # CONTROL DE MOVIMIENTO
                if robot_pos is not None and target_pos is not None and px_per_m > 0:
                    error_vector_px = target_pos - robot_pos
                    error_m_global = error_vector_px / px_per_m
                    distancia = np.linalg.norm(error_m_global)

                    # Transformación de Coordenadas (Mundo -> Robot)
                    dx_robot = error_m_global[0] * math.cos(robot_yaw) + error_m_global[1] * math.sin(robot_yaw)
                    dy_robot = -error_m_global[0] * math.sin(robot_yaw) + error_m_global[1] * math.cos(robot_yaw)
                    
                    twist = Twist()
                    if distancia > DISTANCIA_STOP:
                        twist.linear.x = dx_robot * KP_LIN
                        twist.linear.y = dy_robot * KP_LIN
                    else:
                        twist.linear.x = 0.0
                        twist.linear.y = 0.0

                    self.cmd_vel_pub.publish(twist)
                    
                    cv2.line(frame, tuple(robot_pos), tuple(target_pos), (0, 255, 255), 2)
                    cv2.putText(frame, f"Dist: {distancia:.2f}m", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

        except Exception as e:
            self.get_logger().error(f'Error en loop: {e}')

def main():
    rclpy.init()
    node = EurobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
