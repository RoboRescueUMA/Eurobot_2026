import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D  # Usamos Pose2D para tener X, Y y Theta
import cv2
import numpy as np
import os
import math

# ── FUNCIONES MATEMÁTICAS ─────────────────────────────────────────────────────

def get_marker_corners_3d(center_3d, size):
    cx, cy, cz = center_3d
    hs = size / 2.0
    return np.array([
        [cx - hs, cy + hs, cz], # Top-Left
        [cx + hs, cy + hs, cz], # Top-Right
        [cx + hs, cy - hs, cz], # Bottom-Right
        [cx - hs, cy - hs, cz]  # Bottom-Left
    ], dtype=np.float64)

def estimate_camera_pose(corners_list, ids_list, ground_markers, marker_size, camera_matrix, dist_coeffs):
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
        obj_arr, img_arr, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_SQPNP
    )
    if not success: return None, None

    rvec, tvec = cv2.solvePnPRefineLM(obj_arr, img_arr, camera_matrix, dist_coeffs, rvec, tvec)
    R, _ = cv2.Rodrigues(rvec)
    return R, tvec.reshape(3)

def ray_plane_intersection(pixel_uv, R_cw, t_cw, plane_z, camera_matrix, dist_coeffs):
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
        
        # Publicador usando Pose2D (X, Y, Theta)
        self.pose_pub = self.create_publisher(Pose2D, '/robot_pose', 10)
        
        # ── MODO SIMULACIÓN ──
        # Pon True para leer la foto esta noche. Pon False para la cámara de mañana.
        self.modo_simulacion = True
        
        # ── RUTAS ABSOLUTAS (A prueba de errores de terminal) ──
        self.base_dir = "/home/fabio/eurobot_ws/src/eurobot_vision/eurobot_vision"
        cam_path = os.path.join(self.base_dir, "camera_matrix.npy")
        dist_path = os.path.join(self.base_dir, "dist_coeffs.npy")
        
        if self.modo_simulacion:
            self.get_logger().info("🟡 MODO SIMULACIÓN: Leyendo foto...")
            self.cap = None
        else:
            self.get_logger().info("🟢 MODO CÁMARA: Buscando webcam...")
            self.cap = cv2.VideoCapture(1) # Cambia a 0 si es necesario
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # Configuración ArUco compatible con Ubuntu/ROS2
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()
        
        # Cargar matrices de calibración
        try:
            self.camera_matrix = np.load(cam_path)
            self.dist_coeffs   = np.load(dist_path)
        except FileNotFoundError:
            self.get_logger().error(f"🔴 ERROR: Faltan los .npy en: {self.base_dir}")
            raise

        self.marker_size = 10.0
        self.GROUND_MARKERS_CENTERS = {
            20: np.array([60.0,  140.0, 0.0]),
            21: np.array([240.0, 140.0, 0.0]),
            22: np.array([60.0,   60.0, 0.0]),
            24: np.array([240.0,  60.0, 0.0]),
        }
        
        # ID de tu robot (Cámbialo a 20 si quieres hacer la prueba esta noche con el suelo)
        self.ROBOT_ID = 20
        self.H_ROBOT  = 40.5
        
        self.R_cw = None
        self.t_cw = None
        self.camera_pose_locked = False
        
        self.timer = self.create_timer(0.033, self.timer_callback)

    def timer_callback(self):
        if self.modo_simulacion:
            img_path = os.path.join(self.base_dir, "captura_arena.png")
            frame = cv2.imread(img_path) 
            if frame is None:
                self.get_logger().error(f"🔴 ERROR: No encuentro la foto en: {img_path}")
                return
            frame = cv2.resize(frame, (1920, 1080))
        else:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("Fallo al leer la cámara")
                return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            ids_flat = ids.flatten()
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # 1. Actualizar pose de la cámara (si no está bloqueada)
            if not self.camera_pose_locked:
                R_new, t_new = estimate_camera_pose(
                    corners, ids_flat, self.GROUND_MARKERS_CENTERS, 
                    self.marker_size, self.camera_matrix, self.dist_coeffs
                )
                if R_new is not None:
                    self.R_cw, self.t_cw = R_new, t_new

            # 2. Localizar al robot
            if self.R_cw is not None:
                for i, mid in enumerate(ids_flat):
                    if mid == self.ROBOT_ID:
                        # Extraer esquinas (Top-Left, Top-Right, Bottom-Right, Bottom-Left)
                        tl, tr, br, bl = corners[i][0]
                        
                        # Centro general del marcador para X e Y
                        center_px = corners[i][0].mean(axis=0)
                        
                        # Centros superior e inferior para calcular el ángulo (orientación)
                        top_center_px = (tl + tr) / 2.0
                        bottom_center_px = (bl + br) / 2.0

                        # Proyectar los 3 puntos al plano 3D
                        pos_w = ray_plane_intersection(center_px, self.R_cw, self.t_cw, self.H_ROBOT, self.camera_matrix, self.dist_coeffs)
                        pos_top_w = ray_plane_intersection(top_center_px, self.R_cw, self.t_cw, self.H_ROBOT, self.camera_matrix, self.dist_coeffs)
                        pos_bottom_w = ray_plane_intersection(bottom_center_px, self.R_cw, self.t_cw, self.H_ROBOT, self.camera_matrix, self.dist_coeffs)

                        if pos_w is not None and pos_top_w is not None and pos_bottom_w is not None:
                            
                            # Calcular ángulo Theta (orientación)
                            dx = pos_top_w[0] - pos_bottom_w[0]
                            dy = pos_top_w[1] - pos_bottom_w[1]
                            theta = math.atan2(dy, dx)

                            # ====== PUBLICAR EN ROS 2 ======
                            msg = Pose2D()
                            msg.x = float(pos_w[0])
                            msg.y = float(pos_w[1])
                            msg.theta = float(theta)
                            self.pose_pub.publish(msg)

                            # Dibujar en pantalla
                            cx, cy = int(center_px[0]), int(center_px[1])
                            label = f"X={pos_w[0]:.0f} Y={pos_w[1]:.0f} A={math.degrees(theta):.0f}grados"
                            cv2.putText(frame, label, (cx - 80, cy - 20),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                            cv2.circle(frame, (cx, cy), 6, (0, 255, 0), -1)

        cv2.imshow("Eurobot - Vision", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('l'):
            self.camera_pose_locked = not self.camera_pose_locked
            estado = "BLOQUEADA" if self.camera_pose_locked else "DESBLOQUEADA"
            self.get_logger().info(f"Pose de la cámara {estado}")
        elif key == ord('q'):
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = EurobotVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.modo_simulacion and node.cap is not None:
            node.cap.release()
        cv2.destroyAllWindows()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()