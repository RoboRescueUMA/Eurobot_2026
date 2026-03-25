import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import cv2
import numpy as np

# ── FUNCIONES MATEMÁTICAS ─────────────────────────────────────────────────────

def get_marker_corners_3d(center_3d, size):
    """
    Devuelve las 4 esquinas 3D de un marcador asumiendo que está en el suelo (Z=0)
    y alineado con los ejes X e Y.
    """
    cx, cy, cz = center_3d
    hs = size / 2.0
    return np.array([
        [cx - hs, cy + hs, cz], # Top-Left
        [cx + hs, cy + hs, cz], # Top-Right
        [cx + hs, cy - hs, cz], # Bottom-Right
        [cx - hs, cy - hs, cz]  # Bottom-Left
    ], dtype=np.float64)

def estimate_camera_pose(corners_list, ids_list, ground_markers, marker_size, camera_matrix, dist_coeffs):
    """
    Estima la pose de la cámara usando las 4 ESQUINAS de todos los marcadores.
    """
    obj_pts, img_pts = [], []
    for i, mid in enumerate(ids_list):
        if mid in ground_markers:
            corners_2d = corners_list[i][0] 
            corners_3d = get_marker_corners_3d(ground_markers[mid], marker_size)
            
            for j in range(4):
                img_pts.append(corners_2d[j])
                obj_pts.append(corners_3d[j])

    if len(obj_pts) < 6:
        return None, None

    obj_arr = np.array(obj_pts, dtype=np.float64)
    img_arr = np.array(img_pts, dtype=np.float64)

    success, rvec, tvec = cv2.solvePnP(
        obj_arr, img_arr,
        camera_matrix, dist_coeffs,
        flags=cv2.SOLVEPNP_SQPNP
    )
    
    if not success:
        return None, None

    rvec, tvec = cv2.solvePnPRefineLM(
        obj_arr, img_arr, camera_matrix, dist_coeffs, rvec, tvec
    )

    R, _ = cv2.Rodrigues(rvec)
    return R, tvec.reshape(3)

def ray_plane_intersection(pixel_uv, R_cw, t_cw, plane_z, camera_matrix, dist_coeffs):
    """
    Proyecta el píxel 2D al plano 3D del robot.
    """
    pt_norm = cv2.undistortPoints(
        np.array([[[pixel_uv[0], pixel_uv[1]]]], dtype=np.float64),
        camera_matrix, dist_coeffs
    )[0][0]
    ray_c = np.array([pt_norm[0], pt_norm[1], 1.0])
    R_wc  = R_cw.T
    ray_w = R_wc @ ray_c
    C_w = -(R_wc @ t_cw)

    if abs(ray_w[2]) < 1e-6: return None
    s = (plane_z - C_w[2]) / ray_w[2]
    if s < 0: return None
    return C_w + s * ray_w

# ── NODO DE ROS 2 ─────────────────────────────────────────────────────────────

class EurobotVisionNode(Node):
    def __init__(self):
        super().__init__('eurobot_vision_node')
        
        # 1. Crear el publicador del topic '/robot_pose' (Envía un geometry_msgs/Point)
        self.pose_pub = self.create_publisher(Point, '/robot_pose', 10)
        
        # 2. Configurar la cámara a 1280x720 para mayor rendimiento
        self.cap = cv2.VideoCapture(1) # Cambia a 0 si es la cámara integrada
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # 3. Configuración ArUco
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector   = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        
        # 4. Cargar matrices de calibración (OJO: Asegúrate de que estén en la misma carpeta)
        self.camera_matrix = np.load("camera_matrix.npy")
        self.dist_coeffs   = np.load("dist_coeffs.npy")
        
        # 5. Constantes físicas de la mesa
        self.marker_size = 10.0
        self.GROUND_MARKERS_CENTERS = {
            20: np.array([60.0,  140.0, 0.0]),
            21: np.array([240.0, 140.0, 0.0]),
            22: np.array([60.0,   60.0, 0.0]),
            24: np.array([240.0,  60.0, 0.0]),
        }
        self.ROBOT_ID = 4
        self.H_ROBOT  = 40.5
        
        self.R_cw = None
        self.t_cw = None
        self.camera_pose_locked = False
        
        # 6. Bucle principal de ROS 2: Se ejecuta cada 0.033s (~30 FPS)
        self.timer = self.create_timer(0.033, self.timer_callback)
        self.get_logger().info("Nodo Eurobot iniciado. Buscando marcadores...")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Fallo al leer la cámara")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None:
            ids_flat = ids.flatten()
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # ── A) Actualizar pose de la cámara (si no está bloqueada) ──
            if not self.camera_pose_locked:
                R_new, t_new = estimate_camera_pose(
                    corners, ids_flat, self.GROUND_MARKERS_CENTERS, 
                    self.marker_size, self.camera_matrix, self.dist_coeffs
                )
                if R_new is not None:
                    self.R_cw, self.t_cw = R_new, t_new

            # ── B) Localizar robot y ENVIAR DATOS POR ROS 2 ──
            if self.R_cw is not None:
                for i, mid in enumerate(ids_flat):
                    if mid == self.ROBOT_ID:
                        center_px = corners[i][0].mean(axis=0)

                        pos_w = ray_plane_intersection(
                            center_px, self.R_cw, self.t_cw, self.H_ROBOT,
                            self.camera_matrix, self.dist_coeffs
                        )

                        if pos_w is not None:
                            # ====== MAGIA DE COMUNICACIÓN ======
                            msg = Point()
                            msg.x = float(pos_w[0]) # Coord X
                            msg.y = float(pos_w[1]) # Coord Y
                            msg.z = float(self.H_ROBOT)
                            self.pose_pub.publish(msg) # ¡Se envía por Wi-Fi!
                            # ===================================

                            # Dibujar en pantalla
                            cx, cy = int(center_px[0]), int(center_px[1])
                            label = f"X={pos_w[0]:.1f}  Y={pos_w[1]:.1f} cm"
                            cv2.putText(frame, label, (cx - 60, cy - 20),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                            cv2.circle(frame, (cx, cy), 6, (0, 255, 0), -1)

        cv2.imshow("Eurobot - Localizacion Robot ROS2", frame)
        
        # Controles del teclado en la ventana
        key = cv2.waitKey(1) & 0xFF
        if key == ord('l'):
            self.camera_pose_locked = not self.camera_pose_locked
            estado = "BLOQUEADA" if self.camera_pose_locked else "DESBLOQUEADA"
            self.get_logger().info(f"Pose de la cámara {estado}")
        elif key == ord('q'):
            # Si pulsas Q, se cierra limpiamente
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = EurobotVisionNode()
    try:
        rclpy.spin(node) # Mantiene el nodo vivo ejecutando el timer
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()