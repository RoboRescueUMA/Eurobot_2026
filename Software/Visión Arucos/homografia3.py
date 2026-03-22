import cv2
import numpy as np
import sys

# ------------------------------
# CARGAR CALIBRACIÓN DE LA CÁMARA
# ------------------------------
try:
    camera_matrix = np.load('camera_matrix.npy')
    dist_coeffs = np.load('dist_coeffs.npy')
    print("✅ Calibración cargada")
except FileNotFoundError:
    print("❌ No se encontraron camera_matrix.npy o dist_coeffs.npy.")
    print("   Ejecuta primero el script de calibración con square_size = 0.035")
    sys.exit(1)

# ------------------------------
# CONFIGURACIÓN DE LA CÁMARA
# ------------------------------
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# ------------------------------
# DETECTOR ARUCO
# ------------------------------
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, params)

# ------------------------------
# MARCADORES FIJOS DEL SUELO (coordenadas en METROS)
# ------------------------------
GROUND_IDS = [0, 1, 2, 3]                       # IDs de los 4 marcadores fijos
GROUND_POINTS_M = np.array([
    [0.0, 0.0],   # ID 0
    [0.8, 0.0],   # ID 1
    [0.0, 0.8],   # ID 2
    [0.8, 0.8]    # ID 3
], dtype=np.float32)

# ------------------------------
# MARCADOR DEL ROBOT
# ------------------------------
ROBOT_MARKER_ID = 4                 # ID que has puesto en el robot
GROUND_HEIGHT_M = 0.0               # Altura del suelo (0 metros para proyección al suelo)

# ------------------------------
# VARIABLES GLOBALES
# ------------------------------
homography = None
camera_pose = None                  # (rvec, tvec) de la cámara en el mundo

print("🔄 Esperando a que aparezcan los 4 marcadores fijos...")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # --- CORREGIR DISTORSIÓN (si la calibración es buena) ---
    h, w = frame.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs,
                                                      (w, h), 1, (w, h))
    frame = cv2.undistort(frame, camera_matrix, dist_coeffs, None, newcameramtx)

    # --- DETECTAR MARCADORES ---
    corners, ids, _ = detector.detectMarkers(frame)

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # --- FASE 1: calcular homografía y pose de la cámara (usando los 4 fijos) ---
        if homography is None or camera_pose is None:
            pixeles_fijos = []        # centros en píxeles de los marcadores fijos
            puntos_3d_fijos = []      # sus coordenadas 3D en metros (Z=0)
            for i, id_buscado in enumerate(GROUND_IDS):
                idx = np.where(ids == id_buscado)[0]
                if len(idx) > 0:
                    centro = np.mean(corners[idx[0]][0], axis=0)
                    pixeles_fijos.append(centro)
                    puntos_3d_fijos.append([GROUND_POINTS_M[i][0],
                                            GROUND_POINTS_M[i][1],
                                            0.0])
                else:
                    pixeles_fijos = None
                    break

            if pixeles_fijos is not None and len(pixeles_fijos) == 4:
                # --- Homografía (para objetos en el suelo, en metros) ---
                pixeles_fijos = np.array(pixeles_fijos, dtype=np.float32)
                homography, _ = cv2.findHomography(pixeles_fijos, GROUND_POINTS_M)
                print("✅ Homografía calculada (unidades: metros)")

                # --- Pose de la cámara (extrínsecos) con solvePnP ---
                puntos_3d_fijos = np.array(puntos_3d_fijos, dtype=np.float32)
                _, rvec, tvec = cv2.solvePnP(puntos_3d_fijos, pixeles_fijos,
                                             camera_matrix, dist_coeffs)
                camera_pose = (rvec, tvec)
                print(f"📷 Pose de la cámara -> traslación (m): {tvec.flatten()}")
            else:
                cv2.putText(frame, "Esperando 4 marcadores fijos...", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # --- FASE 2: procesar el marcador del robot ---
        if homography is not None and camera_pose is not None:
            for i, id_detectado in enumerate(ids.flatten()):
                if id_detectado == ROBOT_MARKER_ID:
                    centro_px = np.mean(corners[i][0], axis=0)

                    # ---- CÁLCULO GEOMÉTRICO CON ALTURA CONOCIDA ----
                    # 1. Convertir el píxel (centro del marcador) a un vector dirección en coordenadas de cámara
                    #    (coordenadas normalizadas en el plano Z=1)
                    pixel_homog = np.array([centro_px[0], centro_px[1], 1.0], dtype=np.float32)
                    inv_cam = np.linalg.inv(camera_matrix)
                    ray_cam = inv_cam @ pixel_homog   # dirección en sistema cámara (no normalizada)

                    # 2. Transformar el rayo al sistema del mundo usando la rotación de la cámara
                    R_cam_world, _ = cv2.Rodrigues(camera_pose[0])
                    ray_world = R_cam_world @ ray_cam   # dirección en el mundo (no unitaria)

                    # 3. Posición de la cámara en el mundo (en metros)
                    cam_pos = camera_pose[1].flatten()

                    # 4. Calcular el punto sobre el rayo que está a la altura GROUND_HEIGHT_M
                    #    El suelo es Z=0. Queremos t tal que cam_pos[2] + t * ray_world[2] = GROUND_HEIGHT_M
                    t = (GROUND_HEIGHT_M - cam_pos[2]) / ray_world[2]
                    robot_point = cam_pos + t * ray_world

                    # 5. La posición en el suelo es (robot_point[0], robot_point[1])
                    robot_x_m = robot_point[0]
                    robot_y_m = robot_point[1]
                    robot_z_m = robot_point[2]   # debería ser GROUND_HEIGHT_M

                    # ---- MOSTRAR EN PANTALLA (en cm) ----
                    robot_x_cm = robot_x_m * 100
                    robot_y_cm = robot_y_m * 100
                    texto = f"Robot: ({robot_x_cm:.1f}, {robot_y_cm:.1f}) cm"
                    cv2.putText(frame, texto, tuple(centro_px.astype(int) - [20, 20]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(frame, f"Proyección suelo: {robot_z_m*100:.1f} cm",
                                tuple(centro_px.astype(int) - [20, 0]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

                    # (Opcional) Imprimir en consola
                    print(f"🔵 Proyección suelo -> X={robot_x_m:.2f} m  Y={robot_y_m:.2f} m  Z={robot_z_m:.2f} m")

    # --- MOSTRAR VENTANA ---
    cv2.imshow('Proyección al suelo', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# --- LIMPIAR ---
cap.release()
cv2.destroyAllWindows()