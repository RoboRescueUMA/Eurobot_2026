import cv2
import numpy as np

# ── CONFIGURACIÓN ─────────────────────────────────────────────────────────────
cap = cv2.VideoCapture(1)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector   = cv2.aruco.ArucoDetector(aruco_dict, parameters)

camera_matrix = np.load("camera_matrix.npy")
dist_coeffs   = np.load("dist_coeffs.npy")

marker_size = 10.0  # cm

# Marcadores del suelo: ID -> centro en mundo (X, Y, Z=0)
GROUND_MARKERS_CENTERS = {
    20: np.array([60.0,  140.0, 0.0]),
    21: np.array([240.0, 140.0, 0.0]),
    22: np.array([60.0,   60.0, 0.0]),
    24: np.array([240.0,  60.0, 0.0]),
}

ROBOT_ID = 4
H_ROBOT  = 40.5   # altura del marcador del robot sobre el suelo (cm)

R_cw = None
t_cw = None
camera_pose_locked = False # NUEVO: Para congelar la pose de la cámara

# ── FUNCIONES ─────────────────────────────────────────────────────────────────

def get_marker_corners_3d(center_3d, size):
    """
    Devuelve las 4 esquinas 3D de un marcador asumiendo que está en el suelo (Z=0)
    y alineado con los ejes X e Y.
    El orden de ArUco es: Top-Left, Top-Right, Bottom-Right, Bottom-Left.
    """
    cx, cy, cz = center_3d
    hs = size / 2.0
    return np.array([
        [cx - hs, cy + hs, cz], # Top-Left
        [cx + hs, cy + hs, cz], # Top-Right
        [cx + hs, cy - hs, cz], # Bottom-Right
        [cx - hs, cy - hs, cz]  # Bottom-Left
    ], dtype=np.float64)

def estimate_camera_pose(corners_list, ids_list):
    """
    Estima la pose de la cámara usando las 4 ESQUINAS de todos los marcadores.
    """
    obj_pts, img_pts = [], []
    for i, mid in enumerate(ids_list):
        if mid in GROUND_MARKERS_CENTERS:
            # Puntos 2D en la imagen (las 4 esquinas)
            corners_2d = corners_list[i][0] 
            
            # Puntos 3D en el mundo real (las 4 esquinas)
            corners_3d = get_marker_corners_3d(GROUND_MARKERS_CENTERS[mid], marker_size)
            
            for j in range(4):
                img_pts.append(corners_2d[j])
                obj_pts.append(corners_3d[j])

    # Necesitamos al menos 6 puntos para un PnP estable (1 marcador y medio)
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

    # Refinar con LM (muy recomendable)
    rvec, tvec = cv2.solvePnPRefineLM(
        obj_arr, img_arr, camera_matrix, dist_coeffs, rvec, tvec
    )

    R, _ = cv2.Rodrigues(rvec)
    return R, tvec.reshape(3)

# La función ray_plane_intersection se mantiene EXACTAMENTE IGUAL a la tuya.
def ray_plane_intersection(pixel_uv, R_cw, t_cw, plane_z):
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

# ── BUCLE PRINCIPAL ───────────────────────────────────────────────────────────
print("Iniciando visión Eurobot…")

while True:
    ret, frame = cap.read()
    if not ret: break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        ids_flat = ids.flatten()
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # ── 1) Actualizar pose de la cámara (SOLO SI NO ESTÁ BLOQUEADA) ──────
        if not camera_pose_locked:
            R_new, t_new = estimate_camera_pose(corners, ids_flat)
            if R_new is not None:
                R_cw, t_cw = R_new, t_new
                # Podrías añadir un contador aquí para promediar 30 frames y luego:
                # camera_pose_locked = True
                # print("Cámara calibrada y bloqueada!")

        # ── 2) Localizar robot ────────────────────────────────────────────────
        if R_cw is not None:
            for i, mid in enumerate(ids_flat):
                if mid == ROBOT_ID:
                    center_px = corners[i][0].mean(axis=0)

                    pos_w = ray_plane_intersection(center_px, R_cw, t_cw, H_ROBOT)

                    if pos_w is not None:
                        cx, cy = int(center_px[0]), int(center_px[1])
                        label = f"X={pos_w[0]:.1f}  Y={pos_w[1]:.1f} cm"
                        cv2.putText(frame, label, (cx - 60, cy - 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        cv2.circle(frame, (cx, cy), 6, (0, 255, 0), -1)

    cv2.imshow("Eurobot - Localizacion Robot", frame)
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('l'): # Presiona 'L' para bloquear/desbloquear la pose
        camera_pose_locked = not camera_pose_locked
        print("Pose bloqueada:" if camera_pose_locked else "Pose desbloqueada", camera_pose_locked)

cap.release()
cv2.destroyAllWindows()