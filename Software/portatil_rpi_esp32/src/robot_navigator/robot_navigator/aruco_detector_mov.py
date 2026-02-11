import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point  # Para enviar coordenadas

# --- CONFIGURACIÓN FÍSICA ---
# Esto es lo único que debe ser verdad siempre
TAMAÑO_REAL_ARUCO = 0.05  # 7 cm (0.07 metros)
# ----------------------------

# IDs
MI_EQUIPO = "AZUL"
MI_ROBOT_ID = 1
IDS_AZUL       = [1, 2, 3, 4, 5]
ID_CAJA_AZUL   = [36]

class EurobotDetector(Node):
    def __init__(self):
        super().__init__('eurobot_detector')

        self.aruco_dict_type = cv2.aruco.DICT_4X4_50
        self.bridge = CvBridge()
        
        # QoS = 1 para reducir LAG (importante)
        self.subscription = self.create_subscription(Image, '/zenital/image_raw', self.image_callback, 1)
        self.debug_pub = self.create_publisher(Image, '/zenital/debug', 1)
        
        # Publicamos la posición del objetivo y del robot (en metros)
        self.target_pub = self.create_publisher(Point, '/eurobot/target_pos', 1)
        self.robot_pub = self.create_publisher(Point, '/eurobot/robot_pos', 1)
        
        # Configurar detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.aruco_dict_type)
        try:
            self.aruco_params = cv2.aruco.DetectorParameters()
            # Ajustes para detección difícil (papel/luz)
            self.aruco_params.polygonalApproxAccuracyRate = 0.08 
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.use_new_api = True
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.aruco_params.polygonalApproxAccuracyRate = 0.08
            self.use_new_api = False
        
        self.get_logger().info('✅ Detector Auto-Calibrado LISTO. Usando ArUco de 7cm como referencia.')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Truco para evitar cuelgues de memoria en RPi
            frame = np.ascontiguousarray(frame)
            
            if self.use_new_api:
                corners, ids, _ = self.detector.detectMarkers(frame)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)

            mi_posicion_px = None
            meta_posicion_px = None
            pixels_por_metro = None

            if ids is not None:
                ids = ids.flatten()
                
                # --- FASE 1: AUTO-CALIBRACIÓN Y DIBUJO ---
                for (marker_corner, marker_id) in zip(corners, ids):
                    # Dibujar contorno
                    pts = marker_corner.reshape((-1, 1, 2)).astype(np.int32)
                    cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

                    # Calcular Centro
                    c = marker_corner.reshape((4, 2))
                    cX = int((c[0][0] + c[2][0]) / 2.0)
                    cY = int((c[0][1] + c[2][1]) / 2.0)
                    
                    # --- LA MAGIA: Calcular Escala usando MI ROBOT ---
                    if marker_id == MI_ROBOT_ID:
                        mi_posicion_px = (cX, cY)
                        
                        # Calculamos el perímetro en píxeles
                        perimetro_px = cv2.arcLength(marker_corner, True)
                        # El perímetro real es 4 * 0.07m = 0.28m
                        # Escala = Píxeles / Metros
                        pixels_por_metro = perimetro_px / (TAMAÑO_REAL_ARUCO * 4)
                        
                        label = f"YO (Escala: {int(pixels_por_metro)}px/m)"
                        color = (0, 255, 0)
                        
                    elif marker_id in ID_CAJA_AZUL:
                        meta_posicion_px = (cX, cY)
                        label = "META"
                        color = (255, 0, 0)
                    else:
                        label = f"ID:{marker_id}"
                        color = (0, 255, 255)

                    cv2.putText(frame, label, (cX, cY - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                # --- FASE 2: CÁLCULO DE DISTANCIA REAL Y ENVÍO ---
                if mi_posicion_px and meta_posicion_px and pixels_por_metro:
                    # 1. Calcular diferencias en píxeles
                    dx_px = meta_posicion_px[0] - mi_posicion_px[0]
                    dy_px = meta_posicion_px[1] - mi_posicion_px[1]
                    distancia_px = np.sqrt(dx_px**2 + dy_px**2)
                    
                    # 2. Convertir a Metros
                    distancia_m = distancia_px / pixels_por_metro
                    dx_m = dx_px / pixels_por_metro
                    dy_m = dy_px / pixels_por_metro

                    # 3. ¡ENVIAR AL BUZÓN! (Esto es lo nuevo)
                    msg_point = Point()
                    msg_point.x = dx_m
                    msg_point.y = dy_m
                    msg_point.z = 0.0
                    self.target_pub.publish(msg_point)
                    
                    # 4. Dibujar línea y dato en pantalla
                    cv2.line(frame, mi_posicion_px, meta_posicion_px, (255, 255, 255), 2)
                    cv2.putText(frame, f"{distancia_m:.2f} m", 
                                (int((mi_posicion_px[0]+meta_posicion_px[0])/2), int((mi_posicion_px[1]+meta_posicion_px[1])/2)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
                    
                    self.get_logger().info(f'🚀 Objetivo a {distancia_m:.2f} m | X: {dx_m:.2f}, Y: {dy_m:.2f}')

            # Publicar imagen debug
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = EurobotDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
