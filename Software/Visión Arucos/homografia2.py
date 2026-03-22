import cv2
import numpy as np

# ------------------------------
# CONFIGURACIÓN
# ------------------------------
cap = cv2.VideoCapture(0)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

camera_matrix = np.load("camera_matrix.npy")
dist_coeffs   = np.load("dist_coeffs.npy")

marker_size = 10.0  # cm

# IDs de los marcadores del suelo
id_sup_izq = 0
id_sup_der = 1
id_inf_izq = 2
id_inf_der = 3

# Coordenadas reales de los marcadores del suelo (en cm)
puntos_suelo_3d = {
    id_sup_izq: np.array([0.0,  0.0,  0.0]),
    id_sup_der: np.array([80.0, 0.0,  0.0]),
    id_inf_izq: np.array([0.0,  80.0, 0.0]),
    id_inf_der: np.array([80.0, 80.0, 0.0]),
}

# ID del robot
id_robot = 4
h_robot = 36.0  # altura del marcador sobre el suelo

R_wc = None
t_wc = None

print("Buscando marcadores del suelo para estimar la pose de la cámara...")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        ids = ids.flatten()
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # ------------------------------
        # 1) ESTIMAR POSE DE LA CÁMARA
        # ------------------------------
        if R_wc is None:
            obj_pts = []
            cam_pts = []

            for i, id_detectado in enumerate(ids):
                if id_detectado in puntos_suelo_3d:

                    # Pose del marcador respecto a la cámara
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners[i], marker_size, camera_matrix, dist_coeffs
                    )

                    tvec = tvec[0][0]  # (Xc, Yc, Zc)

                    cam_pts.append(tvec)
                    obj_pts.append(puntos_suelo_3d[id_detectado])

            if len(obj_pts) >= 3:
                obj_pts = np.array(obj_pts, dtype=np.float32)
                cam_pts = np.array(cam_pts, dtype=np.float32)

                # Resolver transformación 3D-3D (suelo -> cámara)
                # Usamos método de Umeyama (SVD)
                centroid_obj = np.mean(obj_pts, axis=0)
                centroid_cam = np.mean(cam_pts, axis=0)

                X = obj_pts - centroid_obj
                Y = cam_pts - centroid_cam

                H = X.T @ Y
                U, S, Vt = np.linalg.svd(H)
                R_wc = Vt.T @ U.T

                if np.linalg.det(R_wc) < 0:
                    Vt[2, :] *= -1
                    R_wc = Vt.T @ U.T

                t_wc = centroid_cam.reshape(3,1) - R_wc @ centroid_obj.reshape(3,1)

                print("Pose cámara (t_wc):", t_wc.T)
                print("Altura estimada cámara:", t_wc[2, 0])

        # ------------------------------
        # 2) ESTIMAR POSE DEL ROBOT
        # ------------------------------
        if R_wc is not None:
            R_cw = R_wc.T
            t_cw = -R_cw @ t_wc

            for i, id_detectado in enumerate(ids):
                if id_detectado == id_robot:

                    rvec_r, tvec_r, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners[i], marker_size, camera_matrix, dist_coeffs
                    )

                    p_c = tvec_r[0][0].reshape(3, 1)

                    # Transformar a coordenadas del suelo
                    p_w = R_cw @ p_c + t_cw

                    # Base del robot (restar altura)
                    base_robot = p_w.copy()
                    base_robot[2, 0] -= h_robot

                    texto = f"Base robot: X={base_robot[0,0]:.1f}  Y={base_robot[1,0]:.1f}  Z={base_robot[2,0]:.1f}"
                    centro_px = np.mean(corners[i][0], axis=0)

                    cv2.putText(frame, texto,
                                (int(centro_px[0]) - 50, int(centro_px[1]) - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    cv2.imshow("Pose 3D estable", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
