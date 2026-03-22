import cv2
import numpy as np

cap = cv2.VideoCapture(0)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

camera_matrix = np.load("camera_matrix.npy")
dist_coeffs   = np.load("dist_coeffs.npy")

marker_size = 10.0  # cm

id_sup_izq = 0
id_sup_der = 1
id_inf_izq = 2
id_inf_der = 3

puntos_suelo_3d = {
    id_sup_izq: np.array([0.0,  0.0,  0.0]),
    id_sup_der: np.array([80.0, 0.0,  0.0]),
    id_inf_izq: np.array([0.0,  80.0, 0.0]),
    id_inf_der: np.array([80.0, 80.0, 0.0]),
}

R_wc = None
t_wc = None

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        ids = ids.flatten()
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        obj_pts = []
        cam_pts = []

        for i, id_detectado in enumerate(ids):
            if id_detectado in puntos_suelo_3d:
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners[i], marker_size, camera_matrix, dist_coeffs
                )
                tvec = tvec[0][0]  # (Xc, Yc, Zc)

                cam_pts.append(tvec)
                obj_pts.append(puntos_suelo_3d[id_detectado])

        if len(obj_pts) >= 3:
            obj_pts = np.array(obj_pts, dtype=np.float32)
            cam_pts = np.array(cam_pts, dtype=np.float32)

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

            print("---- NUEVA MEDIDA ----")
            print("Pose cámara (t_wc):", t_wc.T)
            print("Altura estimada cámara:", t_wc[2, 0])

            # Ahora proyectamos los puntos del suelo al sistema de la cámara y los comparamos
            R_cw = R_wc.T
            t_cw = -R_cw @ t_wc

            for id_m, p_w in puntos_suelo_3d.items():
                p_w = p_w.reshape(3,1)
                p_c = R_wc @ p_w + t_wc
                print(f"Marcador {id_m} en cámara (estimado):", p_c.T)

    cv2.imshow("Debug suelo", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
