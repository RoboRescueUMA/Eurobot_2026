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
  blue_box_pose     - geometry_msgs/Pose2D
  yellow_box_pose   - geometry_msgs/Pose2D
  zenital/debug     - sensor_msgs/Image (imagen con overlays)

Ejecuta en: LAPTOP
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D


# ------------------------------
# IDs ArUco
# ------------------------------
# Marcadores fijos en las 4 esquinas del campo
ID_FIJO_SUP_IZQ = 20   # Origen (0, 0)
ID_FIJO_SUP_DER = 21   # (300, 0)
ID_FIJO_INF_IZQ = 22   # (0, 200)
ID_FIJO_INF_DER = 23   # (300, 200)

IDS_FIJOS = [ID_FIJO_SUP_IZQ, ID_FIJO_SUP_DER, ID_FIJO_INF_IZQ, ID_FIJO_INF_DER]

# Marcadores móviles
ID_ROBOT = 1
ID_CAJA_AZUL = 36
ID_CAJA_AMARILLA = 47

# ------------------------------
# Coordenadas reales del campo (cm)
# Eurobot 2026: 300 cm x 200 cm
# Origen: esquina sup-izq (ArUco 20)
# X+ = derecha, Y+ = abajo
# ------------------------------
PUNTOS_CAMPO_CM = np.array([
    [0.0,   0.0],    # sup_izq  -> ID 20
    [300.0, 0.0],    # sup_der  -> ID 21
    [0.0,   200.0],  # inf_izq  -> ID 22
    [300.0, 200.0],  # inf_der  -> ID 23
], dtype=np.float32)


class FieldLocalizer(Node):
    def __init__(self):
        super().__init__('field_localizer')

        # --- Parámetros ---
        self.declare_parameter('robot_id', ID_ROBOT)
        self.declare_parameter('blue_box_id', ID_CAJA_AZUL)
        self.declare_parameter('yellow_box_id', ID_CAJA_AMARILLA)
        self.declare_parameter('fixed_ids', IDS_FIJOS)
        self.declare_parameter('field_width_cm', 300.0)
        self.declare_parameter('field_height_cm', 200.0)
        self.declare_parameter('homography_update_every_n_frames', 30)

        self.id_robot = self.get_parameter('robot_id').get_parameter_value().integer_value
        self.id_caja_azul = self.get_parameter('blue_box_id').get_parameter_value().integer_value
        self.id_caja_amarilla = self.get_parameter('yellow_box_id').get_parameter_value().integer_value
        self.homography_update_period = self.get_parameter(
            'homography_update_every_n_frames').get_parameter_value().integer_value

        # --- Detector ArUco ---
        self.bridge = CvBridge()
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        try:
            params = cv2.aruco.DetectorParameters()
            params.polygonalApproxAccuracyRate = 0.08
            self.detector = cv2.aruco.ArucoDetector(aruco_dict, params)
            self.use_new_api = True
        except AttributeError:
            self.aruco_dict = aruco_dict
            self.params = cv2.aruco.DetectorParameters_create()
            self.params.polygonalApproxAccuracyRate = 0.08
            self.use_new_api = False

        # --- Estado de homografía ---
        self.homografia = None          # Matriz 3x3 válida más reciente
        self.frame_count = 0            # Para actualización periódica

        # --- Suscriptor / Publicadores ---
        self.sub_image = self.create_subscription(
            Image, 'zenital/image_raw', self.image_callback, 1
        )
        self.pub_debug = self.create_publisher(Image, 'zenital/debug', 1)
        self.pub_robot = self.create_publisher(Pose2D, 'robot_pose', 1)
        self.pub_caja_azul = self.create_publisher(Pose2D, 'blue_box_pose', 1)
        self.pub_caja_amarilla = self.create_publisher(Pose2D, 'yellow_box_pose', 1)

        self.get_logger().info(
            f'FieldLocalizer listo. '
            f'Robot ID={self.id_robot}, Azul={self.id_caja_azul}, Amarilla={self.id_caja_amarilla} | '
            f'Fijos={IDS_FIJOS} | Campo 300x200 cm'
        )

    # ------------------------------------------------------------------
    # Utilidades de homografía
    # ------------------------------------------------------------------

    def _detect(self, frame):
        """Detecta ArUcos y devuelve (corners, ids_flat) o ([], None)."""
        frame_c = np.ascontiguousarray(frame)
        if self.use_new_api:
            corners, ids, _ = self.detector.detectMarkers(frame_c)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                frame_c, self.aruco_dict, parameters=self.params
            )
        if ids is None:
            return corners, None
        return corners, ids.flatten()

    def _try_update_homography(self, corners, ids):
        """
        Intenta recalcular la homografía si los 4 ArUcos fijos son visibles.
        Devuelve True si se actualizó.
        """
        pixeles_fijos = []
        for id_buscado in IDS_FIJOS:
            idx = np.where(ids == id_buscado)[0]
            if len(idx) == 0:
                return False
            # Centro del marcador fijo en píxeles
            centro = np.mean(corners[idx[0]][0], axis=0)
            pixeles_fijos.append(centro)

        pixeles_fijos = np.array(pixeles_fijos, dtype=np.float32)
        H, _ = cv2.findHomography(pixeles_fijos, PUNTOS_CAMPO_CM)
        if H is not None:
            self.homografia = H
            return True
        return False

    def _px_to_cm(self, corners_i):
        """
        Transforma las 4 esquinas de un marcador (shape 4x2, en píxeles)
        a coordenadas de campo (cm) usando la homografía.
        Devuelve (centro_cm, angulo_grados).

        Convenio de ángulo: vector del lado superior (esquina0 -> esquina1)
        en el sistema de coordenadas del campo. 0° = apunta a +X (derecha).
        """
        esquinas_px = corners_i.astype(np.float32).reshape(-1, 1, 2)
        esquinas_cm = cv2.perspectiveTransform(esquinas_px, self.homografia).reshape(-1, 2)

        centro_cm = np.mean(esquinas_cm, axis=0)

        # Lado superior: esquina 0 (top-left) -> esquina 1 (top-right)
        lado = esquinas_cm[1] - esquinas_cm[0]
        angulo_grados = float(np.degrees(np.arctan2(lado[1], lado[0])))

        return centro_cm, angulo_grados

    # ------------------------------------------------------------------
    # Callback principal
    # ------------------------------------------------------------------

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error cv_bridge: {e}')
            return

        corners, ids = self._detect(frame)

        # --- Dibujar marcadores detectados ---
        if ids is not None and len(corners) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids.reshape(-1, 1))

        # --- Actualizar homografía (cada N frames o si aún no la tenemos) ---
        self.frame_count += 1
        if ids is not None:
            if self.homografia is None or (self.frame_count % self.homography_update_period == 0):
                updated = self._try_update_homography(corners, ids)
                if updated:
                    self.get_logger().info('Homografia actualizada con los 4 ArUcos fijos.')

        # --- Sin homografía: avisar y publicar debug ---
        if self.homografia is None:
            cv2.putText(
                frame,
                'Esperando 4 ArUcos fijos (IDs 20,21,22,23)...',
                (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2
            )
            self._publish_debug(frame)
            return

        # --- Con homografía: procesar marcadores móviles ---
        if ids is None:
            self._publish_debug(frame)
            return

        for i, marker_id in enumerate(ids):
            marker_corners = corners[i][0]  # shape (4, 2)
            centro_px = np.mean(marker_corners, axis=0).astype(int)

            if marker_id in IDS_FIJOS:
                # Solo etiquetar los fijos en la imagen
                cv2.putText(
                    frame,
                    f'F{marker_id}',
                    tuple(centro_px - [0, 25]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 100, 0), 2
                )
                continue

            # Transformar a coordenadas de campo
            centro_cm, angulo_deg = self._px_to_cm(marker_corners)
            x_cm = float(centro_cm[0])
            y_cm = float(centro_cm[1])

            # --- Publicar según ID ---
            pose_msg = Pose2D()
            pose_msg.x = x_cm
            pose_msg.y = y_cm
            pose_msg.theta = angulo_deg

            if marker_id == self.id_robot:
                self.pub_robot.publish(pose_msg)
                label = f'ROBOT ({x_cm:.0f},{y_cm:.0f}) {angulo_deg:.0f}deg'
                color = (0, 255, 0)
                self.get_logger().debug(
                    f'Robot: X={x_cm:.1f}cm Y={y_cm:.1f}cm theta={angulo_deg:.1f}deg'
                )

            elif marker_id == self.id_caja_azul:
                self.pub_caja_azul.publish(pose_msg)
                label = f'AZUL ({x_cm:.0f},{y_cm:.0f}) {angulo_deg:.0f}deg'
                color = (255, 80, 0)

            elif marker_id == self.id_caja_amarilla:
                self.pub_caja_amarilla.publish(pose_msg)
                label = f'AMARILLA ({x_cm:.0f},{y_cm:.0f}) {angulo_deg:.0f}deg'
                color = (0, 220, 220)

            else:
                # Marcador desconocido: mostrar en imagen pero no publicar
                label = f'ID:{marker_id} ({x_cm:.0f},{y_cm:.0f})'
                color = (128, 128, 128)

            # Overlay en imagen de debug
            cv2.putText(
                frame, label,
                tuple(centro_px - [0, 25]),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2
            )

            # Flecha de orientación (en píxeles, 50px de longitud)
            angulo_rad = np.radians(angulo_deg)
            # Nota: la flecha se dibuja en el espacio imagen, solo orientativa
            arrow_px = (
                int(centro_px[0] + 50 * np.cos(angulo_rad)),
                int(centro_px[1] + 50 * np.sin(angulo_rad))
            )
            cv2.arrowedLine(frame, tuple(centro_px), arrow_px, color, 2, tipLength=0.3)

        self._publish_debug(frame)

    def _publish_debug(self, frame):
        try:
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        except Exception as e:
            self.get_logger().error(f'Error publicando debug: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = FieldLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Deteniendo field_localizer...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
