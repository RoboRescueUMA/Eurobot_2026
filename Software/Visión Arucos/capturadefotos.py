import cv2
import os

# ----------------------------------------------
# CONFIGURACIÓN
# ----------------------------------------------
# Índice de la cámara (0 para la primera webcam, 1 para la segunda, etc.)
CAMARA_ID = 0

# Carpeta donde se guardarán las fotos
CARPETA_DESTINO = "fotos_calibracion"

# Resolución deseada (opcional, comenta si no quieres fijarla)
ANCHO = 1920
ALTO = 1080

# ----------------------------------------------
# INICIALIZACIÓN
# ----------------------------------------------
# Abrir la cámara
cap = cv2.VideoCapture(CAMARA_ID, cv2.CAP_DSHOW)  # CAP_DSHOW mejora compatibilidad en Windows

# Fijar resolución (si la cámara lo permite)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, ANCHO)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, ALTO)

# Crear la carpeta de destino si no existe
os.makedirs(CARPETA_DESTINO, exist_ok=True)

contador = 0
print(f"📁 Las fotos se guardarán en: {CARPETA_DESTINO}")
print("🎯 Pulsa ESPACIO para guardar una foto, ESC para salir.")

# ----------------------------------------------
# BUCLE PRINCIPAL
# ----------------------------------------------
while True:
    # Leer un frame de la cámara
    ret, frame = cap.read()
    if not ret:
        print("❌ Error al leer la cámara")
        break

    # Corregir distorsión
    h, w = frame.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w,h), 1, (w,h))
    frame = cv2.undistort(frame, camera_matrix, dist_coeffs, None, newcameramtx)
    # Mostrar el frame en una ventana
    cv2.imshow("Captura para calibración", frame)

    # Esperar 1 ms y detectar tecla pulsada
    key = cv2.waitKey(1) & 0xFF

    if key == 27:  # Tecla ESC
        print("👋 Saliendo...")
        break
    elif key == 32:  # Tecla ESPACIO
        # Generar nombre de archivo único
        nombre_archivo = os.path.join(CARPETA_DESTINO, f"foto_{contador:03d}.jpg")
        # Guardar la imagen
        cv2.imwrite(nombre_archivo, frame)
        print(f"✅ Guardada: {nombre_archivo}")
        contador += 1

# ----------------------------------------------
# LIBERAR RECURSOS
# ----------------------------------------------
cap.release()
cv2.destroyAllWindows()