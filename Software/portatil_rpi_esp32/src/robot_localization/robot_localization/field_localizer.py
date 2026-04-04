#!/usr/bin/env python3
"""
field_localizer.py - Localización absoluta en campo Eurobot mediante homografía ArUco

Flujo:
  1. Suscribe a /roborescue/zenital/image_raw
  2. Detecta los 4 ArUcos fijos en las esquinas del campo (IDs 20-23)
  3. Calcula (y actualiza) la homografía píxeles -> cm del campo
  4. Transforma las posiciones/orientaciones de robot y cajas a coordenadas absolutas
  5. Publica Pose2D en coordenadas del campo (X, Y en cm, theta en grados)

Sistema de coordenadas del campo:
  Origen = esquina superior-izquierda (ArUco 20)
  X+ = hacia la derecha  (hacia ArUco 21)
  Y+ = hacia abajo       (hacia ArUco 22)

Topics publicados (en namespace /roborescue/):
  robot_pose        - geometry_msgs/Pose2D  (X cm, Y cm, theta grados)
  blue_box_pose     - geometry_msgs/Pose2D (mejor candidato azul)
  yellow_box_pose   - geometry_msgs/Pose2D (mejor candidato amarillo)
  blue_box_poses    - geometry_msgs/PoseArray (todos los candidatos en metros)
  yellow_box_poses  - geometry_msgs/PoseArray (todos los candidatos en metros)
  zenital/debug     - sensor_msgs/Image (imagen con overlays)

Ejecuta en: LAPTOP
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D, Pose, PoseArray
import math
import os
from collections import deque, defaultdict


# ------------------------------
# IDs ArUco
# ------------------------------
# Marcadores fijos en las 4 esquinas del campo
ID_FIJO_SUP_IZQ = 20  # Origen (0, 0)
ID_FIJO_SUP_DER = 21  # (300, 0)
ID_FIJO_INF_IZQ = 22  # (0, 200)
ID_FIJO_INF_DER = 23  # (300, 200)

IDS_FIJOS = [ID_FIJO_SUP_IZQ, ID_FIJO_SUP_DER, ID_FIJO_INF_IZQ, ID_FIJO_INF_DER]

# Marcadores móviles
ID_ROBOT = 1
ID_CAJA_AZUL = 36
ID_CAJA_AMARILLA = 47

# ------------------------------
# Coordenadas reales del campo (cm)
# Eurobot 2026: 300 cm x 200 cm
# Origen: esquina inf-izq del campo (0,0)
# X+ = derecha, Y+ = arriba
# ArUcos centrados en el campo (no en esquinas exactas)
# ------------------------------
PUNTOS_CAMPO_CM = np.array(
    [
        [60.0, 140.0],  # sup_izq  -> ID 20
        [240.0, 140.0],  # sup_der  -> ID 21
        [60.0, 60.0],  # inf_izq  -> ID 22
        [240.0, 60.0],  # inf_der  -> ID 23
    ],
    dtype=np.float32,
)

# Coordenadas 3D de los centros de los ArUcos en el suelo
GROUND_MARKERS_CENTERS = {
    20: np.array([60.0, 140.0, 0.0]),
    21: np.array([240.0, 140.0, 0.0]),
    22: np.array([60.0, 60.0, 0.0]),
    23: np.array([240.0, 60.0, 0.0]),
}

MARKER_SIZE_CM = 10.0


def get_marker_corners_3d(center_3d, size_cm):
    cx, cy, cz = center_3d
    half = size_cm / 2.0
    return np.array(
        [
            [cx - half, cy + half, cz],  # Top-Left
            [cx + half, cy + half, cz],  # Top-Right
            [cx + half, cy - half, cz],  # Bottom-Right
            [cx - half, cy - half, cz],  # Bottom-Left
        ],
        dtype=np.float64,
    )


def estimate_camera_pose(
    corners_list, ids_list, marker_size, camera_matrix, dist_coeffs
):
    obj_pts, img_pts = [], []
    for idx, marker_id in enumerate(ids_list):
        if marker_id not in GROUND_MARKERS_CENTERS:
            continue
        corners_2d = corners_list[idx][0]
        corners_3d = get_marker_corners_3d(
            GROUND_MARKERS_CENTERS[marker_id], marker_size
        )
        img_pts.extend(corners_2d)
        obj_pts.extend(corners_3d)

    if len(obj_pts) < 6:
        return None, None

    obj_arr = np.array(obj_pts, dtype=np.float64)
    img_arr = np.array(img_pts, dtype=np.float64)
    success, rvec, tvec = cv2.solvePnP(
        obj_arr, img_arr, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_SQPNP
    )
    if not success:
        return None, None

    rvec, tvec = cv2.solvePnPRefineLM(
        obj_arr, img_arr, camera_matrix, dist_coeffs, rvec, tvec
    )
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    return rotation_matrix, tvec.reshape(3)


def ray_plane_intersection(
    pixel_uv, rotation, translation, plane_z, camera_matrix, dist_coeffs
):
    pt_norm = cv2.undistortPoints(
        np.array([[[pixel_uv[0], pixel_uv[1]]]], dtype=np.float64),
        camera_matrix,
        dist_coeffs,
    )[0][0]
    ray_c = np.array([pt_norm[0], pt_norm[1], 1.0])
    rot_wc = rotation.T
    ray_w = rot_wc @ ray_c
    camera_center = -(rot_wc @ translation)

    if abs(ray_w[2]) < 1e-6:
        return None

    s = (plane_z - camera_center[2]) / ray_w[2]
    if s < 0:
        return None
    return camera_center + s * ray_w


class FieldLocalizer(Node):
    def __init__(self):
        default_namespace = os.environ.get("ROS_NAMESPACE") or "/roborescue"
        super().__init__("field_localizer", namespace=default_namespace)

        # --- Parámetros ---
        self.declare_parameter("robot_id", ID_ROBOT)
        self.declare_parameter("blue_box_id", ID_CAJA_AZUL)
        self.declare_parameter("yellow_box_id", ID_CAJA_AMARILLA)
        self.declare_parameter("fixed_ids", IDS_FIJOS)
        self.declare_parameter("field_width_cm", 300.0)
        self.declare_parameter("field_height_cm", 200.0)
        self.declare_parameter("homography_update_every_n_frames", 30)
        self.declare_parameter("robot_marker_height_cm", 37.0)
        self.declare_parameter("box_marker_height_cm", 3.0)
        self.declare_parameter("modo_simulacion", False)
        self.declare_parameter("camera_index", 2)

        self.id_robot = (
            self.get_parameter("robot_id").get_parameter_value().integer_value
        )
        self.id_caja_azul = (
            self.get_parameter("blue_box_id").get_parameter_value().integer_value
        )
        self.id_caja_amarilla = (
            self.get_parameter("yellow_box_id").get_parameter_value().integer_value
        )
        # Uso futuro: permitir actualización periódica si se decide liberar la pose.
        self.homography_update_period = (
            self.get_parameter("homography_update_every_n_frames")
            .get_parameter_value()
            .integer_value
        )
        self.robot_marker_height = (
            self.get_parameter("robot_marker_height_cm")
            .get_parameter_value()
            .double_value
        )
        self.box_marker_height = (
            self.get_parameter("box_marker_height_cm")
            .get_parameter_value()
            .double_value
        )
        self.modo_simulacion = (
            self.get_parameter("modo_simulacion").get_parameter_value().bool_value
        )
        self.camera_index_declared = (
            self.get_parameter("camera_index").get_parameter_value().integer_value
        )

        # --- Detector ArUco ---
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.use_new_api = False
        self.detector = None
        self.params = cv2.aruco.DetectorParameters_create()
        self.params.minMarkerPerimeterRate = 0.01
        self.params.maxMarkerPerimeterRate = 4.5
        self.params.adaptiveThreshWinSizeMin = 3
        self.params.adaptiveThreshWinSizeMax = 61
        self.params.adaptiveThreshWinSizeStep = 4
        self.params.adaptiveThreshConstant = 7
        self.params.polygonalApproxAccuracyRate = 0.03
        self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.params.cornerRefinementMinAccuracy = 0.02
        self.params.detectInvertedMarker = True

        # --- Calibración / configuración ---
        pkg_dir = os.path.dirname(__file__)
        calib_dir = os.path.join(pkg_dir, "calibration")
        self.camera_matrix_path = os.path.join(calib_dir, "camera_matrix.npy")
        self.dist_coeffs_path = os.path.join(calib_dir, "dist_coeffs.npy")
        self.sim_image_path = os.path.join(calib_dir, "captura_arena.png")
        try:
            self.camera_matrix = np.load(self.camera_matrix_path)
            self.dist_coeffs = np.load(self.dist_coeffs_path)
        except FileNotFoundError as exc:
            raise RuntimeError(
                "Calibración no encontrada. Coloca camera_matrix.npy y dist_coeffs.npy en"
                f" {calib_dir}"
            ) from exc

        self.desired_width = 1920
        self.desired_height = 1080
        self.desired_fps = 30
        self.desired_fourcc = cv2.VideoWriter_fourcc(*"MJPG")

        if self.modo_simulacion:
            self.cap = None
            self.get_logger().info(
                "FieldLocalizer en modo simulación. Leyendo imagen fija."
            )
        else:
            self.cap = self._open_preferred_camera()

        self.camera_rotation = None
        self.camera_translation = None
        self.camera_pose_locked = False

        # --- Estado de homografía / calibración ---
        self.homografia = None  # Matriz 3x3 válida más reciente
        self.frame_count = 0  # Para actualización periódica
        self.calibration_complete = False
        self.calibration_samples_needed = 7
        self.calibration_samples_collected = 0
        self.homography_samples = []
        self.rotation_samples = []
        self.translation_samples = []
        self.last_calibration_sample_time = 0.0
        self.calibration_sample_interval_sec = 0.25

        # --- Estabilización de poses por ID ---
        self.pose_history = {}
        self.pose_history_size = 5
        self.pose_consensus_threshold = 3
        self.pose_position_epsilon = 5.0  # cm
        self.pose_angle_epsilon = 10.0  # grados

        # --- Seguimiento multi-marcador (IDs compartidos por varias cajas) ---
        self.multi_marker_ids = {self.id_caja_azul, self.id_caja_amarilla}
        self.multi_tracks = defaultdict(list)
        self.primary_track_per_marker = {}
        self.last_published_pose = {}
        self.last_pose_publish_time = {}
        self.track_id_counter = 0
        self.track_assignment_distance_cm = 25.0
        self.track_timeout_sec = 0.75
        self.pose_array_frame_id = "field_cm"
        self.stable_pose_hold_sec = 0.7

        # --- Suscriptor / Publicadores ---
        self.sub_image = self.create_subscription(
            Image, "zenital/image_raw", self.image_callback, 1
        )
        self.timer = self.create_timer(0.01, self.loop_callback)
        self.pub_debug = self.create_publisher(Image, "zenital/debug", 1)
        self.pub_robot = self.create_publisher(Pose2D, "robot_pose", 1)
        self.pub_caja_azul = self.create_publisher(Pose2D, "blue_box_pose", 1)
        self.pub_caja_amarilla = self.create_publisher(Pose2D, "yellow_box_pose", 1)
        self.pub_caja_azul_array = self.create_publisher(PoseArray, "blue_box_poses", 1)
        self.pub_caja_amarilla_array = self.create_publisher(
            PoseArray, "yellow_box_poses", 1
        )

        self.get_logger().info(
            "FieldLocalizer listo. "
            f"Robot ID={self.id_robot}, Azul={self.id_caja_azul}, Amarilla={self.id_caja_amarilla} | "
            f"Fijos={IDS_FIJOS} | Campo 300x200 cm"
        )

    # ------------------------------------------------------------------
    # Utilidades de homografía
    # ------------------------------------------------------------------

    def _detect(self, frame):
        """Detecta ArUcos y devuelve (corners, ids_flat) o ([], None)."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        frame_c = cv2.equalizeHist(gray)
        frame_c = np.ascontiguousarray(frame_c)
        if self.use_new_api:
            corners, ids, _ = self.detector.detectMarkers(frame_c)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                frame_c, self.aruco_dict, parameters=getattr(self, "params", None)
            )

        if ids is None:
            return corners, None

        return corners, ids.flatten()

    def _fixed_markers_visible(self, ids):
        if ids is None:
            return False
        present = set(ids.tolist())
        return all(marker_id in present for marker_id in IDS_FIJOS)

    def _get_fixed_marker_pixels(self, corners, ids):
        if ids is None:
            return None
        pixeles_fijos = []
        for marker_id in IDS_FIJOS:
            idx = np.where(ids == marker_id)[0]
            if len(idx) == 0:
                return None
            centro = np.mean(corners[idx[0]][0], axis=0)
            pixeles_fijos.append(centro)
        return np.array(pixeles_fijos, dtype=np.float32)

    def _compute_homography_from_fixed(self, corners, ids):
        pixeles_fijos = self._get_fixed_marker_pixels(corners, ids)
        if pixeles_fijos is None:
            return None
        H, _ = cv2.findHomography(pixeles_fijos, PUNTOS_CAMPO_CM)
        return H

    def _maybe_collect_calibration_sample(self, corners, ids):
        if not self._fixed_markers_visible(ids):
            return False

        now_sec = self._now_seconds()
        if (
            self.calibration_samples_collected > 0
            and (now_sec - self.last_calibration_sample_time)
            < self.calibration_sample_interval_sec
        ):
            return False

        H = self._compute_homography_from_fixed(corners, ids)
        if H is None:
            return False

        rotation, translation = estimate_camera_pose(
            corners,
            ids,
            MARKER_SIZE_CM,
            self.camera_matrix,
            self.dist_coeffs,
        )
        if rotation is None:
            return False

        rvec, _ = cv2.Rodrigues(rotation)

        self.homography_samples.append(H)
        self.rotation_samples.append(rvec.reshape(3))
        self.translation_samples.append(translation)

        self.homografia = H
        self.camera_rotation = rotation
        self.camera_translation = translation

        self.calibration_samples_collected += 1
        self.last_calibration_sample_time = now_sec

        self.get_logger().info(
            f"Muestra de calibracion {self.calibration_samples_collected}/{self.calibration_samples_needed}"
        )

        if self.calibration_samples_collected >= self.calibration_samples_needed:
            self._finalize_calibration()

        return True

    def _finalize_calibration(self):
        H_stack = np.stack(self.homography_samples, axis=0)
        H_avg = np.mean(H_stack, axis=0)
        if abs(H_avg[2, 2]) > 1e-9:
            H_avg /= H_avg[2, 2]
        self.homografia = H_avg

        rvecs = np.stack(self.rotation_samples, axis=0)
        avg_rvec = np.mean(rvecs, axis=0).reshape(3, 1)
        rotation_avg, _ = cv2.Rodrigues(avg_rvec)
        self.camera_rotation = rotation_avg

        translations = np.stack(self.translation_samples, axis=0)
        self.camera_translation = np.mean(translations, axis=0)

        self.calibration_complete = True
        self.camera_pose_locked = True

        self.get_logger().info("Calibracion completada: homografia y pose bloqueadas.")

    def _now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _px_to_cm(self, corners_i):
        """
        Transforma las 4 esquinas de un marcador (shape 4x2, en píxeles)
        a coordenadas de campo (cm) usando la homografía.
        Devuelve (centro_cm, angulo_grados).

        Convenio de ángulo: vector del lado superior (esquina0 -> esquina1)
        en el sistema de coordenadas del campo. 0° = apunta a +X (derecha).
        """
        esquinas_px = corners_i.astype(np.float32).reshape(-1, 1, 2)
        esquinas_cm = cv2.perspectiveTransform(esquinas_px, self.homografia).reshape(
            -1, 2
        )

        centro_cm = np.mean(esquinas_cm, axis=0)

        # Lado superior: esquina 0 (top-left) -> esquina 1 (top-right)
        lado = esquinas_cm[1] - esquinas_cm[0]
        angulo_grados = float(np.degrees(np.arctan2(lado[1], lado[0])))

        return centro_cm, angulo_grados

    # ------------------------------------------------------------------
    # Callback principal
    # ------------------------------------------------------------------

    def loop_callback(self):
        frame = self._get_frame()
        if frame is None:
            return
        self._process_frame(frame)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error cv_bridge: {e}")
            return
        self._process_frame(frame)

    def _get_frame(self):
        if self.modo_simulacion:
            frame = cv2.imread(self.sim_image_path)
            if frame is None:
                self.get_logger().error(f"No se puede cargar {self.sim_image_path}")
                return None
            return cv2.resize(frame, (1920, 1080))
        if self.cap is None:
            return None
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn(
                "No se pudo leer la cámara, revisa el índice o la conexión"
            )
            return None
        h, w = frame.shape[:2]
        if w < self.desired_width or h < self.desired_height:
            self.get_logger().warn(
                f"Resolucion degradada ({w}x{h}), reconfigurando camara"
            )
            self._configure_camera(self.cap)
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("Reconfiguracion de camara fallida")
                return None
        return frame

    def _process_frame(self, frame):

        corners, ids = self._detect(frame)

        # --- Dibujar marcadores detectados ---
        if ids is not None and len(corners) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids.reshape(-1, 1))

        ids_flat = ids if ids is not None else None

        # --- Calibración inicial ---
        if not self.calibration_complete:
            self._maybe_collect_calibration_sample(corners, ids_flat)

            status = f"Calibrando homografia ({self.calibration_samples_collected}/{self.calibration_samples_needed})"
            if not self._fixed_markers_visible(ids_flat):
                status = "Esperando ver los 4 ArUcos fijos (IDs 20,21,22,23)..."

            cv2.putText(
                frame,
                status,
                (10, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 0, 255),
                2,
            )
            self._publish_debug(frame)
            return

        # --- Con homografía fijada: procesar marcadores móviles ---
        if ids_flat is None:
            self._publish_debug(frame)
            return

        self._update_camera_pose(corners, ids_flat)

        detections = []
        for i, marker_id in enumerate(ids_flat):
            marker_corners = corners[i][0]  # shape (4, 2)
            centro_px = np.mean(marker_corners, axis=0).astype(int)

            if marker_id in IDS_FIJOS:
                continue

            marker_height = self.robot_marker_height
            if marker_id in (self.id_caja_azul, self.id_caja_amarilla):
                marker_height = self.box_marker_height

            pose_cm = self._pnp_localization(marker_corners, marker_height)

            if pose_cm is not None:
                x_cm, y_cm, theta_deg = pose_cm
            else:
                centro_cm, angulo_deg = self._px_to_cm(marker_corners)
                x_cm = float(centro_cm[0])
                y_cm = float(centro_cm[1])
                theta_deg = angulo_deg

            pose_msg = Pose2D()
            pose_msg.x = x_cm
            pose_msg.y = y_cm
            pose_msg.theta = theta_deg
            detections.append(
                {
                    "id": marker_id,
                    "pose": pose_msg,
                    "theta": theta_deg,
                    "centro_px": centro_px,
                }
            )

        self._update_multi_tracks(detections)
        for marker_id in self.multi_marker_ids:
            self._publish_multi_marker_pose(marker_id)

        for det in detections:
            marker_id = det["id"]
            pose_msg = det["pose"]
            theta_deg = det["theta"]
            centro_px = det["centro_px"]
            label, color = self._handle_pose_publication(marker_id, pose_msg)

            # Overlay en imagen de debug
            cv2.putText(
                frame,
                label,
                tuple(centro_px - [0, 25]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
            )

            # Flecha de orientación (en píxeles, 50px de longitud)
            angulo_rad = np.radians(theta_deg)
            # Nota: la flecha se dibuja en el espacio imagen, solo orientativa
            arrow_px = (
                int(centro_px[0] + 50 * np.cos(angulo_rad)),
                int(centro_px[1] + 50 * np.sin(angulo_rad)),
            )
            cv2.arrowedLine(frame, tuple(centro_px), arrow_px, color, 2, tipLength=0.3)

        self._publish_debug(frame)

    def _publish_stable_pose(self, marker_id, pose_msg, publisher):
        history = self.pose_history.setdefault(
            marker_id, deque(maxlen=self.pose_history_size)
        )
        history.append(pose_msg)

        now = self._now_seconds()

        if len(history) < self.pose_consensus_threshold:
            if self._should_republish(marker_id, now):
                publisher.publish(pose_msg)
                self.last_pose_publish_time[marker_id] = now
            return pose_msg

        base = history[-1]
        count = 0
        sum_x, sum_y, sum_theta = 0.0, 0.0, 0.0
        for pose in history:
            if (
                abs(pose.x - base.x) <= self.pose_position_epsilon
                and abs(pose.y - base.y) <= self.pose_position_epsilon
                and abs((pose.theta - base.theta + 180) % 360 - 180)
                <= self.pose_angle_epsilon
            ):
                count += 1
                sum_x += pose.x
                sum_y += pose.y
                sum_theta += pose.theta

        if count >= self.pose_consensus_threshold:
            stable_pose = Pose2D()
            stable_pose.x = sum_x / count
            stable_pose.y = sum_y / count
            stable_pose.theta = sum_theta / count
            publisher.publish(stable_pose)
            self.last_pose_publish_time[marker_id] = now
            return stable_pose

        if self._should_republish(marker_id, now):
            publisher.publish(pose_msg)
            self.last_pose_publish_time[marker_id] = now
        return pose_msg

    def _should_republish(self, marker_id, now):
        last_time = self.last_pose_publish_time.get(marker_id)
        if last_time is None:
            return True
        return (now - last_time) >= self.stable_pose_hold_sec

    def _handle_pose_publication(self, marker_id, pose_msg):
        if marker_id == self.id_robot:
            self._publish_stable_pose(marker_id, pose_msg, self.pub_robot)
            label = f"ROBOT ({pose_msg.x:.0f},{pose_msg.y:.0f}) {pose_msg.theta:.0f}deg"
            color = (0, 255, 0)
            self.get_logger().debug(
                f"Robot: X={pose_msg.x:.1f}cm Y={pose_msg.y:.1f}cm theta={pose_msg.theta:.1f}deg"
            )
            return label, color

        if marker_id in self.multi_marker_ids:
            return self._label_for_marker(marker_id, pose_msg)

        label = f"ID:{marker_id} ({pose_msg.x:.0f},{pose_msg.y:.0f})"
        color = (128, 128, 128)
        return label, color

    def _label_for_marker(self, marker_id, pose_msg):
        if marker_id == self.id_caja_azul:
            return (
                f"AZUL ({pose_msg.x:.0f},{pose_msg.y:.0f}) {pose_msg.theta:.0f}deg",
                (255, 80, 0),
            )
        return (
            f"AMARILLA ({pose_msg.x:.0f},{pose_msg.y:.0f}) {pose_msg.theta:.0f}deg",
            (0, 220, 220),
        )

    def _publish_multi_marker_pose(self, marker_id):
        tracks = self.multi_tracks.get(marker_id, [])
        best_track = None
        best_age = float("inf")
        now = self._now_seconds()

        pose_array = PoseArray()
        pose_array.header.frame_id = self.pose_array_frame_id
        pose_array.header.stamp = self.get_clock().now().to_msg()

        for track in tracks:
            if now - track["last_seen"] > self.track_timeout_sec:
                continue
            pose_array.poses.append(self._pose2d_to_pose(track["pose"]))
            if track["track_id"] == self.primary_track_per_marker.get(marker_id):
                best_track = track
                best_age = track["first_seen"]
            elif best_track is None or track["first_seen"] < best_age:
                best_track = track
                best_age = track["first_seen"]

        if not pose_array.poses:
            return

        if marker_id == self.id_caja_azul:
            self.pub_caja_azul_array.publish(pose_array)
            stable_pub = self.pub_caja_azul
        else:
            self.pub_caja_amarilla_array.publish(pose_array)
            stable_pub = self.pub_caja_amarilla

        if best_track is None:
            return

        self.primary_track_per_marker[marker_id] = best_track["track_id"]
        stable_pose = self._publish_stable_pose(
            marker_id, best_track["pose"], stable_pub
        )
        self.last_published_pose[marker_id] = stable_pose

    def _update_multi_tracks(self, detections):
        now = self._now_seconds()
        for marker_id in self.multi_marker_ids:
            self.multi_tracks[marker_id] = [
                track
                for track in self.multi_tracks.get(marker_id, [])
                if now - track["last_seen"] <= self.track_timeout_sec
            ]

        for det in detections:
            marker_id = det["id"]
            pose_msg = det["pose"]
            if marker_id not in self.multi_marker_ids:
                continue

            tracks = self.multi_tracks[marker_id]
            assigned_track = None
            for track in tracks:
                dx = pose_msg.x - track["pose"].x
                dy = pose_msg.y - track["pose"].y
                dist = math.hypot(dx, dy)
                if dist <= self.track_assignment_distance_cm:
                    assigned_track = track
                    break

            if assigned_track is None:
                self.track_id_counter += 1
                assigned_track = {
                    "track_id": self.track_id_counter,
                    "pose": Pose2D(),
                    "first_seen": now,
                    "last_seen": now,
                }
                tracks.append(assigned_track)

            assigned_track["pose"].x = pose_msg.x
            assigned_track["pose"].y = pose_msg.y
            assigned_track["pose"].theta = pose_msg.theta
            assigned_track["last_seen"] = now

    def _pose2d_to_pose(self, pose2d):
        pose = Pose()
        pose.position.x = pose2d.x / 100.0
        pose.position.y = pose2d.y / 100.0
        pose.position.z = 0.0
        yaw_rad = math.radians(pose2d.theta)
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = math.sin(yaw_rad / 2.0)
        pose.orientation.w = math.cos(yaw_rad / 2.0)
        return pose

    def _publish_debug(self, frame):
        try:
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        except Exception as e:
            self.get_logger().error(f"Error publicando debug: {e}")

    def _autodetect_camera_index(self):
        try:
            video_devices = sorted(
                dev for dev in os.listdir("/dev") if dev.startswith("video")
            )
        except FileNotFoundError as exc:
            raise RuntimeError(
                "Directorio /dev no accesible para listar cámaras"
            ) from exc

        indices = []
        for dev in video_devices:
            suffix = dev[len("video") :]
            if suffix.isdigit():
                indices.append(int(suffix))

        if not indices:
            raise RuntimeError(
                "No se encontraron dispositivos /dev/video*; especifica camera_index"
            )

        preferred = [idx for idx in sorted(indices) if idx > 0]
        index = preferred[0] if preferred else sorted(indices)[0]

        self.get_logger().warn(
            f"camera_index no definido; usando /dev/video{index}. "
            "Pasa -p camera_index:=N si deseas otro dispositivo."
        )
        return index

    def _open_preferred_camera(self):
        indices_to_try = []
        if self.camera_index_declared >= 0:
            indices_to_try.append(self.camera_index_declared)
        else:
            autodetected = self._autodetect_camera_index()
            indices_to_try.append(autodetected)
            if autodetected != 2:
                indices_to_try.append(2)
            if autodetected != 3:
                indices_to_try.append(3)

        for idx in indices_to_try:
            cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
            self._configure_camera(cap)
            if cap.isOpened():
                self.get_logger().info(f"FieldLocalizer usando cámara index {idx}")
                self.camera_index = idx
                return cap
            cap.release()
            self.get_logger().warn(
                f"No se pudo abrir /dev/video{idx}, probando otro índice..."
            )

        raise RuntimeError(
            "No se pudo abrir ninguna cámara. Revisa la conexión USB o especifica camera_index explícitamente."
        )

    def _configure_camera(self, cap):
        cap.set(cv2.CAP_PROP_FOURCC, self.desired_fourcc)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.desired_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.desired_height)
        cap.set(cv2.CAP_PROP_FPS, self.desired_fps)

    def _update_camera_pose(self, corners, ids):
        if self.camera_pose_locked:
            return
        rotation, translation = estimate_camera_pose(
            corners,
            ids,
            MARKER_SIZE_CM,
            self.camera_matrix,
            self.dist_coeffs,
        )
        if rotation is not None:
            self.camera_rotation = rotation
            self.camera_translation = translation
            self.get_logger().debug("Pose de cámara actualizada mediante PnP")

    def _pnp_localization(self, marker_corners, marker_height_cm):
        if self.camera_rotation is None or self.camera_translation is None:
            return None

        corners = marker_corners
        center_px = corners.mean(axis=0)
        top_center = (corners[0] + corners[1]) / 2.0
        bottom_center = (corners[2] + corners[3]) / 2.0

        pos = ray_plane_intersection(
            center_px,
            self.camera_rotation,
            self.camera_translation,
            marker_height_cm,
            self.camera_matrix,
            self.dist_coeffs,
        )
        top_pos = ray_plane_intersection(
            top_center,
            self.camera_rotation,
            self.camera_translation,
            marker_height_cm,
            self.camera_matrix,
            self.dist_coeffs,
        )
        bottom_pos = ray_plane_intersection(
            bottom_center,
            self.camera_rotation,
            self.camera_translation,
            marker_height_cm,
            self.camera_matrix,
            self.dist_coeffs,
        )

        if pos is None or top_pos is None or bottom_pos is None:
            return None

        dx = top_pos[0] - bottom_pos[0]
        dy = top_pos[1] - bottom_pos[1]
        theta = math.degrees(math.atan2(dy, dx))
        return float(pos[0]), float(pos[1]), float(theta)


def main(args=None):
    rclpy.init(args=args)
    node = FieldLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Deteniendo field_localizer...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
