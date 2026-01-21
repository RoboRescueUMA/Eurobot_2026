import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class EurobotDetector(Node):
    def __init__(self):
        super().__init__('eurobot_detector')

        # Configuración básica
        self.aruco_dict_type = cv2.aruco.DICT_4X4_50
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(Image, '/zenital/image_raw', self.image_callback, 10)
        self.debug_pub = self.create_publisher(Image, '/zenital/debug', 10)
        
        # --- PREPARACIÓN DEL DETECTOR ---
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.aruco_dict_type)
        try:
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.use_new_api = True
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.use_new_api = False
        
        self.get_logger().info('✅ Detector ARRANCADO (Modo Dibujo Manual). Esperando vídeo...')

    def image_callback(self, msg):
        try:
            # 1. Aseguramos que la imagen está bien en memoria
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = np.ascontiguousarray(frame) # Truco para evitar errores raros de memoria
            
            # 2. Detección
            if self.use_new_api:
                corners, ids, rejected = self.detector.detectMarkers(frame)
            else:
                corners, ids, rejected = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)

            # 3. Dibujado MANUAL (Esto no falla nunca)
            if ids is not None:
                ids = ids.flatten() # Aplanar lista de IDs
                
                for (marker_corner, marker_id) in zip(corners, ids):
                    # Convertir coordenadas a enteros
                    pts = marker_corner.reshape((-1, 1, 2)).astype(np.int32)
                    
                    # Dibujar cuadrado (Verde)
                    cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
                    
                    # Calcular centro para poner el texto
                    c = marker_corner.reshape((4, 2))
                    (topLeft, topRight, bottomRight, bottomLeft) = c
                    cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                    cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                    
                    # Escribir ID
                    cv2.putText(frame, f"ID: {marker_id}", (cX, cY - 15), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    self.get_logger().info(f'👀 ¡VEO ALGO! ID detectado: {marker_id}')
            
            # 4. Publicar
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

        except Exception as e:
            # Si falla, imprimimos el error pero NO cerramos el programa
            self.get_logger().error(f'⚠️ Error recuperable: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = EurobotDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
