import cv2
import numpy as np

# ------------------------------
# CONFIGURACIÓN: CÁMARA (cambia la URL por la de tu móvil)
# ------------------------------
cap = cv2.VideoCapture(0)

# ------------------------------
# CONFIGURACIÓN: ARUCO (API actualizada)
# ------------------------------
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# ------------------------------
# COORDENADAS REALES DE TUS 4 ARUCOS (en cm)
# ------------------------------
# IDs de tus marcadores fijos (cámbialos si es necesario)
id_sup_izq = 0
id_sup_der = 1
id_inf_izq = 2
id_inf_der = 3

# Posiciones reales en (X, Y) en cm (mídelas con cinta métrica)
puntos_reales = np.array([
    [0, 0],      # sup_izq
    [80, 0],     # sup_der
    [0, 80],     # inf_izq
    [80, 80]     # inf_der
], dtype=np.float32)

homografia = None

print("🔄 Buscando los 4 ArUcos fijos para calcular homografía...")

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Error con la cámara")
        break

    # Detectar marcadores usando el detector
    corners, ids, _ = detector.detectMarkers(frame)

    if ids is not None:
        # Dibujar los marcadores detectados
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # ----- INTENTAR CALCULAR HOMOGRAFÍA SI AÚN NO SE HA HECHO -----
        if homografia is None:
            pixeles_fijos = []
            for id_buscado in [id_sup_izq, id_sup_der, id_inf_izq, id_inf_der]:
                idx = np.where(ids == id_buscado)[0]
                if len(idx) > 0:
                    centro = np.mean(corners[idx[0]][0], axis=0)
                    pixeles_fijos.append(centro)
                else:
                    pixeles_fijos = None
                    break

            if pixeles_fijos is not None and len(pixeles_fijos) == 4:
                pixeles_fijos = np.array(pixeles_fijos, dtype=np.float32)
                homografia, _ = cv2.findHomography(pixeles_fijos, puntos_reales)
                print("✅ ¡Homografía calculada!")
            else:
                cv2.putText(frame, "Esperando 4 marcadores fijos...", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

        # ----- SI YA TENEMOS HOMOGRAFÍA, PROCESAMOS TODOS LOS MARCADORES -----
        if homografia is not None:
            for i, id_detectado in enumerate(ids.flatten()):
                # Centro en píxeles (para dibujar)
                centro_px = np.mean(corners[i][0], axis=0)

                if id_detectado not in [id_sup_izq, id_sup_der, id_inf_izq, id_inf_der]:
                    # --- NUEVO: Calcular posición y orientación en el mundo real ---
                    
                    # 1. Obtener las cuatro esquinas del marcador en píxeles
                    esquinas_px = corners[i][0].astype(np.float32)  # shape (4,2)
                    
                    # 2. Transformar cada esquina a coordenadas mundo usando la homografía
                    #    (necesitamos reshape a (4,1,2) para perspectiveTransform)
                    esquinas_px_reshape = esquinas_px.reshape(-1, 1, 2)
                    esquinas_mundo = cv2.perspectiveTransform(esquinas_px_reshape, homografia).reshape(-1, 2)
                    
                    # 3. Calcular el centro en el mundo (promedio de las cuatro esquinas)
                    centro_mundo = np.mean(esquinas_mundo, axis=0)
                    
                    # 4. Calcular la orientación: vector del lado superior (esquina 0 -> esquina 1)
                    #    Según OpenCV, las esquinas se devuelven en orden: top-left, top-right, bottom-right, bottom-left
                    lado_superior = esquinas_mundo[1] - esquinas_mundo[0]
                    angulo_rad = np.arctan2(lado_superior[1], lado_superior[0])
                    angulo_grados = np.degrees(angulo_rad)
                    
                    # 5. Mostrar posición y ángulo en la imagen
                    texto_pos = f"ID:{id_detectado} ({centro_mundo[0]:.1f}, {centro_mundo[1]:.1f}) cm"
                    texto_ang = f"ang: {angulo_grados:.1f} deg"
                    
                    # Dibujar posición (verde) y ángulo (azul) cerca del marcador
                    cv2.putText(frame, texto_pos, tuple(centro_px.astype(int) - [20, 20]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                    cv2.putText(frame, texto_ang, tuple(centro_px.astype(int) - [20, 0]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,0), 2)
                    
                    # Opcional: dibujar las esquinas transformadas para depurar
                    # for j, (x, y) in enumerate(esquinas_mundo):
                    #     cv2.circle(frame, (int(x), int(y)), 3, (0,255,255), -1)
                else:
                    # Marcadores fijos: solo mostramos su ID
                    cv2.putText(frame, f"Fijo {id_detectado}", tuple(centro_px.astype(int) - [20, -20]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,0), 2)

    cv2.imshow('Homografía en vivo', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()