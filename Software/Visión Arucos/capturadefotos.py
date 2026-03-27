import cv2
import os

# ----------------------------------------------
# CONFIGURACIÓN
# ----------------------------------------------
CAMARA_ID = 1  # 0 o 1 según la webcam
CARPETA_DESTINO = "fotos_calibracion"
ANCHO = 1920
ALTO = 1080

# ----------------------------------------------
# INICIALIZACIÓN
# ----------------------------------------------
cap = cv2.VideoCapture(CAMARA_ID, cv2.CAP_DSHOW)

# Forzar resolución máxima para detectar ArUcos pequeños después
cap.set(cv2.CAP_PROP_FRAME_WIDTH, ANCHO)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, ALTO)

os.makedirs(CARPETA_DESTINO, exist_ok=True)

# CONFIGURACIÓN DE VENTANA (Para evitar el efecto de recorte)
cv2.namedWindow("Captura para calibración", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Captura para calibración", 1280, 720) # Solo cambia como lo ves tú, no la foto

contador = 0
print(f"📁 Fotos en: {CARPETA_DESTINO}")
print("🎯 ESPACIO: Guardar | ESC: Salir")

# ----------------------------------------------
# BUCLE PRINCIPAL
# ----------------------------------------------
while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Error al leer la cámara")
        break

    # IMPORTANTE: NO aplicar undistort aquí. 
    # Para calibrar necesitamos la imagen deformada original.

    cv2.imshow("Captura para calibración", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == 27:  # ESC
        break
    elif key == 32:  # ESPACIO
        nombre_archivo = os.path.join(CARPETA_DESTINO, f"foto_{contador:03d}.jpg")
        cv2.imwrite(nombre_archivo, frame)
        print(f"✅ Guardada {contador:03d} a resolución {frame.shape[1]}x{frame.shape[0]}")
        contador += 1

cap.release()
cv2.destroyAllWindows()