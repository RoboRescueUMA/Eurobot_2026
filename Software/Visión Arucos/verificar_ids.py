import cv2
import numpy as np
import sys

# ------------------------------
# CARGAR PARÁMETROS DE CALIBRACIÓN (opcional, pero recomendado)
# ------------------------------
try:
    camera_matrix = np.load('camera_matrix.npy')
    dist_coeffs = np.load('dist_coeffs.npy')
    print("✅ Parámetros de calibración cargados.")
    use_undistort = True
except FileNotFoundError:
    print("⚠️  No se encontró calibración. Se usará la imagen sin undistort.")
    use_undistort = False

# ------------------------------
# CONFIGURACIÓN DE CÁMARA
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
# MARCADORES FIJOS DEL SUELO
# ------------------------------
GROUND_IDS = [0, 1, 2, 3]            # IDs de los 4 marcadores fijos
puntos_reales_cm = np.array([
    [0, 0],      # ID 0
    [80, 0],     # ID 1
    [0, 80],     # ID 2
    [80, 80]     # ID 3
], dtype=np.float32)

# Marcador que pondrás en el suelo para probar (por ejemplo, el del robot)
TEST_MARKER_ID = 4

homografia = None

print("🔄 Buscando los 4 ArUcos fijos...")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Corregir distorsión si está disponible
    if use_undistort:
        h, w = frame.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
        frame = cv2.undistort(frame, camera_matrix, dist_coeffs, None, newcameramtx)

    corners, ids, _ = detector.detectMarkers(frame)

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # --- CÁLCULO DE HOMOGRAFÍA ---
        if homografia is None:
            pixeles_fijos = []
            for id_buscado in GROUND_IDS:
                idx = np.where(ids == id_buscado)[0]
                if len(idx) > 0:
                    centro = np.mean(corners[idx[0]][0], axis=0)
                    pixeles_fijos.append(centro)
                else:
                    pixeles_fijos = None
                    break
            if pixeles_fijos is not None and len(pixeles_fijos) == 4:
                pixeles_fijos = np.array(pixeles_fijos, dtype=np.float32)
                homografia, _ = cv2.findHomography(pixeles_fijos, puntos_reales_cm)
                print("✅ Homografía calculada")

        # --- SI YA TENEMOS HOMOGRAFÍA, PROBAR CON EL MARCADOR DE PRUEBA ---
        if homografia is not None:
            for i, id_detectado in enumerate(ids.flatten()):
                centro_px = np.mean(corners[i][0], axis=0)
                # Mostrar ID en la imagen
                cv2.putText(frame, str(id_detectado), tuple(centro_px.astype(int) - [10, 10]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # Si es el marcador de prueba (colocado en el suelo)
                if id_detectado == TEST_MARKER_ID:
                    # Convertir sus píxeles a coordenadas del mundo usando la homografía
                    punto_pixel = np.array([[[centro_px[0], centro_px[1]]]], dtype=np.float32)
                    punto_real = cv2.perspectiveTransform(punto_pixel, homografia)[0][0]
                    texto = f"({punto_real[0]:.1f}, {punto_real[1]:.1f}) cm"
                    cv2.putText(frame, texto, tuple(centro_px.astype(int) + [10, 10]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    print(f"Posición del marcador {TEST_MARKER_ID}: {texto}")

    cv2.imshow('Homografía en suelo', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()