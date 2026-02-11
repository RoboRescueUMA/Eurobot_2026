import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

# --- CONFIGURACIÓN FÍSICA ---
TAMANO_ARUCO_REAL = 0.05  # 5 cm (0.05 metros) - NO CAMBIAR
# ----------------------------

# IDs de ArUco
ID_MI_ROBOT = 1
ID_CAJA_AZUL = 36
ID_CAJA_AMARILLA = 47  # TODO: Verificar este ID

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        # Declarar parámetros
        self.declare_parameter('robot_id', ID_MI_ROBOT)
        self.declare_parameter('blue_box_id', ID_CAJA_AZUL)
        self.declare_parameter('yellow_box_id', ID_CAJA_AMARILLA)
        
        # Obtener parámetros
        self.id_robot = self.get_parameter('robot_id').get_parameter_value().integer_value
        self.id_caja_azul = self.get_parameter('blue_box_id').get_parameter_value().integer_value
        self.id_caja_amarilla = self.get_parameter('yellow_box_id').get_parameter_value().integer_value
        
        # Configuración ArUco
        self.tipo_dict_aruco = cv2.aruco.DICT_4X4_50
        self.bridge = CvBridge()
        
        # Suscribirse al stream de cámara (QoS=1 para reducir lag)
        # Usar nombre relativo (sin /) porque el namespace se aplica automáticamente
        self.subscription = self.create_subscription(
            Image, 
            'zenital/image_raw',  # Sin / inicial - se convierte en /robot1/zenital/image_raw
            self.callback_imagen, 
            1
        )
        
        # Publicadores (nombres relativos)
        self.pub_debug = self.create_publisher(Image, 'zenital/debug', 1)
        self.pub_pos_robot = self.create_publisher(Point, 'robot_pos', 1)
        self.pub_pos_caja_azul = self.create_publisher(Point, 'blue_box_pos', 1)
        self.pub_pos_caja_amarilla = self.create_publisher(Point, 'yellow_box_pos', 1)
        
        # Configurar detector
        self.dict_aruco = cv2.aruco.getPredefinedDictionary(self.tipo_dict_aruco)
        try:
            self.params_aruco = cv2.aruco.DetectorParameters()
            self.params_aruco.polygonalApproxAccuracyRate = 0.08  # Mejor detección con papel/iluminación
            self.detector = cv2.aruco.ArucoDetector(self.dict_aruco, self.params_aruco)
            self.usar_api_nueva = True
        except AttributeError:
            self.params_aruco = cv2.aruco.DetectorParameters_create()
            self.params_aruco.polygonalApproxAccuracyRate = 0.08
            self.usar_api_nueva = False
        
        self.get_logger().info(f'✅ Detector ArUco listo. Robot ID: {self.id_robot}, '
                               f'Caja azul: {self.id_caja_azul}, Caja amarilla: {self.id_caja_amarilla}')

    def callback_imagen(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = np.ascontiguousarray(frame)  # Prevenir problemas de memoria en RPi
            
            # Detectar marcadores
            if self.usar_api_nueva:
                esquinas, ids, _ = self.detector.detectMarkers(frame)
            else:
                esquinas, ids, _ = cv2.aruco.detectMarkers(frame, self.dict_aruco, parameters=self.params_aruco)

            pos_robot_px = None
            pos_caja_azul_px = None
            pos_caja_amarilla_px = None
            pixeles_por_metro = None

            if ids is not None:
                ids = ids.flatten()
                
                # FASE 1: AUTO-CALIBRACIÓN Y DIBUJO
                for (esquina_marcador, id_marcador) in zip(esquinas, ids):
                    # Dibujar contorno
                    pts = esquina_marcador.reshape((-1, 1, 2)).astype(np.int32)
                    cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

                    # Calcular centro
                    c = esquina_marcador.reshape((4, 2))
                    cX = int((c[0][0] + c[2][0]) / 2.0)
                    cY = int((c[0][1] + c[2][1]) / 2.0)
                    
                    # AUTO-CALIBRACIÓN: Calcular escala usando marcador del robot
                    if id_marcador == self.id_robot:
                        pos_robot_px = (cX, cY)
                        
                        # Calcular perímetro en píxeles
                        perimetro_px = cv2.arcLength(esquina_marcador, True)
                        # El perímetro real es 4 * 0.05m = 0.20m
                        pixeles_por_metro = perimetro_px / (TAMANO_ARUCO_REAL * 4)
                        
                        etiqueta = f"ROBOT (Escala: {int(pixeles_por_metro)}px/m)"
                        color = (0, 255, 0)
                        
                    elif id_marcador == self.id_caja_azul:
                        pos_caja_azul_px = (cX, cY)
                        etiqueta = "CAJA AZUL"
                        color = (255, 0, 0)
                        
                    elif id_marcador == self.id_caja_amarilla:
                        pos_caja_amarilla_px = (cX, cY)
                        etiqueta = "CAJA AMARILLA"
                        color = (0, 255, 255)
                        
                    else:
                        etiqueta = f"ID:{id_marcador}"
                        color = (128, 128, 128)

                    cv2.putText(frame, etiqueta, (cX, cY - 15), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                # FASE 2: CALCULAR POSICIONES REALES Y PUBLICAR
                if pos_robot_px and pixeles_por_metro:
                    # Publicar posición del robot (siempre en origen 0,0 en marco del robot)
                    msg_robot = Point()
                    msg_robot.x = 0.0
                    msg_robot.y = 0.0
                    msg_robot.z = 0.0
                    self.pub_pos_robot.publish(msg_robot)
                    
                    # Publicar posición de caja azul (relativa al robot)
                    if pos_caja_azul_px:
                        dx_px = pos_caja_azul_px[0] - pos_robot_px[0]
                        dy_px = pos_caja_azul_px[1] - pos_robot_px[1]
                        distancia_px = np.sqrt(dx_px**2 + dy_px**2)
                        
                        dx_m = dx_px / pixeles_por_metro
                        dy_m = dy_px / pixeles_por_metro
                        distancia_m = distancia_px / pixeles_por_metro
                        
                        msg_azul = Point()
                        msg_azul.x = dx_m
                        msg_azul.y = dy_m
                        msg_azul.z = 0.0
                        self.pub_pos_caja_azul.publish(msg_azul)
                        
                        # Dibujar línea y distancia
                        cv2.line(frame, pos_robot_px, pos_caja_azul_px, (255, 0, 0), 2)
                        medio_x = int((pos_robot_px[0] + pos_caja_azul_px[0]) / 2)
                        medio_y = int((pos_robot_px[1] + pos_caja_azul_px[1]) / 2)
                        cv2.putText(frame, f"{distancia_m:.2f}m", (medio_x, medio_y),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                        
                        self.get_logger().info(f'🔵 Caja azul a {distancia_m:.2f}m | X: {dx_m:.2f}, Y: {dy_m:.2f}')
                    
                    # Publicar posición de caja amarilla (relativa al robot)
                    if pos_caja_amarilla_px:
                        dx_px = pos_caja_amarilla_px[0] - pos_robot_px[0]
                        dy_px = pos_caja_amarilla_px[1] - pos_robot_px[1]
                        distancia_px = np.sqrt(dx_px**2 + dy_px**2)
                        
                        dx_m = dx_px / pixeles_por_metro
                        dy_m = dy_px / pixeles_por_metro
                        distancia_m = distancia_px / pixeles_por_metro
                        
                        msg_amarillo = Point()
                        msg_amarillo.x = dx_m
                        msg_amarillo.y = dy_m
                        msg_amarillo.z = 0.0
                        self.pub_pos_caja_amarilla.publish(msg_amarillo)
                        
                        # Dibujar línea y distancia
                        cv2.line(frame, pos_robot_px, pos_caja_amarilla_px, (0, 255, 255), 2)
                        medio_x = int((pos_robot_px[0] + pos_caja_amarilla_px[0]) / 2)
                        medio_y = int((pos_robot_px[1] + pos_caja_amarilla_px[1]) / 2)
                        cv2.putText(frame, f"{distancia_m:.2f}m", (medio_x, medio_y),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                        
                        self.get_logger().info(f'🟡 Caja amarilla a {distancia_m:.2f}m | X: {dx_m:.2f}, Y: {dy_m:.2f}')

            # Publicar imagen de debug
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

        except Exception as e:
            self.get_logger().error(f'❌ Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
