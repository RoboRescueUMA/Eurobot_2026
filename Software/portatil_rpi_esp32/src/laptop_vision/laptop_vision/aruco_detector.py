import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D

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
            'zenital/image_raw',  # Sin / inicial - se convierte en /roborescue/zenital/image_raw
            self.callback_imagen, 
            1
        )
        
        # Publicadores (nombres relativos)
        self.pub_debug = self.create_publisher(Image, 'zenital/debug', 1)
        self.pub_pos_robot = self.create_publisher(Pose2D, 'robot_pose', 1)
        self.pub_pos_caja_azul = self.create_publisher(Pose2D, 'blue_box_pose', 1)
        self.pub_pos_caja_amarilla = self.create_publisher(Pose2D, 'yellow_box_pose', 1)
        
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

    def calcular_orientacion(self, esquinas):
        """
        Calcular orientación (ángulo theta) del marcador ArUco
        Las esquinas están ordenadas: [0, 1, 2, 3] en sentido horario
        Esquina 0 (superior izquierda) indica "adelante" del marcador
        
        Returns: ángulo en radianes como float de Python (0 = apunta a la derecha en imagen, positivo = antihorario)
        """
        # Vector desde centro hacia esquina 0 (dirección "adelante" del ArUco)
        centro = np.mean(esquinas, axis=0)
        vector_adelante = esquinas[0] - centro
        
        # atan2 da el ángulo en radianes respecto al eje X positivo (derecha)
        # En coordenadas de imagen: X=derecha, Y=abajo
        theta = np.arctan2(vector_adelante[1], vector_adelante[0])
        
        # Convertir de numpy.float64 a float de Python para compatibilidad con ROS2
        return float(theta)
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
            esquinas_robot = None
            pos_caja_azul_px = None
            esquinas_caja_azul = None
            pos_caja_amarilla_px = None
            esquinas_caja_amarilla = None
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
                    
                    # Dibujar esquina 0 (indica orientación "adelante" del ArUco)
                    esquina_0 = (int(c[0][0]), int(c[0][1]))
                    cv2.circle(frame, esquina_0, 8, (0, 0, 255), -1)  # Círculo rojo en esquina 0
                    
                    # AUTO-CALIBRACIÓN: Calcular escala usando marcador del robot
                    if id_marcador == self.id_robot:
                        pos_robot_px = (cX, cY)
                        esquinas_robot = c
                        
                        # Calcular perímetro en píxeles
                        perimetro_px = cv2.arcLength(esquina_marcador, True)
                        # El perímetro real es 4 * 0.05m = 0.20m
                        pixeles_por_metro = perimetro_px / (TAMANO_ARUCO_REAL * 4)
                        
                        etiqueta = f"ROBOT (Escala: {int(pixeles_por_metro)}px/m)"
                        color = (0, 255, 0)
                        
                    elif id_marcador == self.id_caja_azul:
                        pos_caja_azul_px = (cX, cY)
                        esquinas_caja_azul = c
                        etiqueta = "CAJA AZUL"
                        color = (255, 0, 0)
                        
                    elif id_marcador == self.id_caja_amarilla:
                        pos_caja_amarilla_px = (cX, cY)
                        esquinas_caja_amarilla = c
                        etiqueta = "CAJA AMARILLA"
                        color = (0, 255, 255)
                        
                    else:
                        etiqueta = f"ID:{id_marcador}"
                        color = (128, 128, 128)

                    cv2.putText(frame, etiqueta, (cX, cY - 15), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                # FASE 2: CALCULAR POSICIONES, ORIENTACIONES Y PUBLICAR
                if pos_robot_px and esquinas_robot is not None and pixeles_por_metro:
                    # Calcular orientación del robot
                    theta_robot = self.calcular_orientacion(esquinas_robot)
                    
                    # Publicar pose del robot (siempre en origen 0,0,0 en marco del robot)
                    msg_robot = Pose2D()
                    msg_robot.x = 0.0
                    msg_robot.y = 0.0
                    msg_robot.theta = 0.0  # En su propio marco de referencia
                    self.pub_pos_robot.publish(msg_robot)
                    
                    # Dibujar flecha indicando orientación del robot
                    arrow_length = 50
                    arrow_end_x = int(pos_robot_px[0] + arrow_length * np.cos(theta_robot))
                    arrow_end_y = int(pos_robot_px[1] + arrow_length * np.sin(theta_robot))
                    cv2.arrowedLine(frame, pos_robot_px, (arrow_end_x, arrow_end_y), 
                                   (0, 255, 0), 3, tipLength=0.3)
                    
                    # Publicar posición de caja azul (relativa al robot)
                    if pos_caja_azul_px and esquinas_caja_azul is not None:
                        # Posición relativa en coordenadas de imagen
                        dx_px = pos_caja_azul_px[0] - pos_robot_px[0]
                        dy_px = pos_caja_azul_px[1] - pos_robot_px[1]
                        distancia_px = np.sqrt(dx_px**2 + dy_px**2)
                        
                        # Convertir a metros
                        dx_m = dx_px / pixeles_por_metro
                        dy_m = dy_px / pixeles_por_metro
                        distancia_m = distancia_px / pixeles_por_metro
                        
                        # Orientación de la caja
                        theta_caja = self.calcular_orientacion(esquinas_caja_azul)
                        # Orientación relativa al robot (convertir a float de Python)
                        theta_rel = float(theta_caja - theta_robot)
                        
                        msg_azul = Pose2D()
                        msg_azul.x = float(dx_m)
                        msg_azul.y = float(dy_m)
                        msg_azul.theta = theta_rel
                        self.pub_pos_caja_azul.publish(msg_azul)
                        
                        # Dibujar línea y distancia
                        cv2.line(frame, pos_robot_px, pos_caja_azul_px, (255, 0, 0), 2)
                        medio_x = int((pos_robot_px[0] + pos_caja_azul_px[0]) / 2)
                        medio_y = int((pos_robot_px[1] + pos_caja_azul_px[1]) / 2)
                        cv2.putText(frame, f"{distancia_m:.2f}m", (medio_x, medio_y),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                        
                        # Dibujar flecha de orientación de la caja
                        arrow_length_box = 40
                        arrow_end_x = int(pos_caja_azul_px[0] + arrow_length_box * np.cos(theta_caja))
                        arrow_end_y = int(pos_caja_azul_px[1] + arrow_length_box * np.sin(theta_caja))
                        cv2.arrowedLine(frame, pos_caja_azul_px, (arrow_end_x, arrow_end_y), 
                                       (255, 0, 0), 2, tipLength=0.3)
                        
                        self.get_logger().info(
                            f'🔵 Caja azul a {distancia_m:.2f}m | X: {dx_m:.2f}, Y: {dy_m:.2f}, '
                            f'Theta: {np.degrees(theta_rel):.1f}°'
                        )
                    
                    # Publicar posición de caja amarilla (relativa al robot)
                    if pos_caja_amarilla_px and esquinas_caja_amarilla is not None:
                        # Posición relativa en coordenadas de imagen
                        dx_px = pos_caja_amarilla_px[0] - pos_robot_px[0]
                        dy_px = pos_caja_amarilla_px[1] - pos_robot_px[1]
                        distancia_px = np.sqrt(dx_px**2 + dy_px**2)
                        
                        # Convertir a metros
                        dx_m = dx_px / pixeles_por_metro
                        dy_m = dy_px / pixeles_por_metro
                        distancia_m = distancia_px / pixeles_por_metro
                        
                        # Orientación de la caja
                        theta_caja = self.calcular_orientacion(esquinas_caja_amarilla)
                        # Orientación relativa al robot (convertir a float de Python)
                        theta_rel = float(theta_caja - theta_robot)
                        
                        msg_amarillo = Pose2D()
                        msg_amarillo.x = float(dx_m)
                        msg_amarillo.y = float(dy_m)
                        msg_amarillo.theta = theta_rel
                        self.pub_pos_caja_amarilla.publish(msg_amarillo)
                        
                        # Dibujar línea y distancia
                        cv2.line(frame, pos_robot_px, pos_caja_amarilla_px, (0, 255, 255), 2)
                        medio_x = int((pos_robot_px[0] + pos_caja_amarilla_px[0]) / 2)
                        medio_y = int((pos_robot_px[1] + pos_caja_amarilla_px[1]) / 2)
                        cv2.putText(frame, f"{distancia_m:.2f}m", (medio_x, medio_y),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                        
                        # Dibujar flecha de orientación de la caja
                        arrow_length_box = 40
                        arrow_end_x = int(pos_caja_amarilla_px[0] + arrow_length_box * np.cos(theta_caja))
                        arrow_end_y = int(pos_caja_amarilla_px[1] + arrow_length_box * np.sin(theta_caja))
                        cv2.arrowedLine(frame, pos_caja_amarilla_px, (arrow_end_x, arrow_end_y), 
                                       (0, 255, 255), 2, tipLength=0.3)
                        
                        self.get_logger().info(
                            f'🟡 Caja amarilla a {distancia_m:.2f}m | X: {dx_m:.2f}, Y: {dy_m:.2f}, '
                            f'Theta: {np.degrees(theta_rel):.1f}°'
                        )

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
