import cv2
import numpy as np
import glob
import os

# ---------------------------------------------------------
# 1. CONFIGURACIÓN (¡Verifica estos datos con tu papel!)
# ---------------------------------------------------------
CARPETA_FOTOS = "fotos_calibracion2"
CARPETA_SALIDA = "fotos_calibracion_detectadas2"

# ¡OJO! Número de INTERSECCIONES internas, no cuadrados. (Columnas, Filas)
pattern_size = (7, 10) 
# Tamaño del lado de un cuadrado en metros (0.035 = 3.5 cm)
square_size = 0.035 

os.makedirs(CARPETA_SALIDA, exist_ok=True)

# ---------------------------------------------------------
# 2. PREPARACIÓN
# ---------------------------------------------------------
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
objp *= square_size

objpoints = [] # Puntos 3D en el espacio del mundo real
imgpoints = [] # Puntos 2D en el plano de la imagen

images = glob.glob(os.path.join(CARPETA_FOTOS, '*.jpg')) + \
         glob.glob(os.path.join(CARPETA_FOTOS, '*.png'))

print(f"📂 Buscando imágenes en: {CARPETA_FOTOS}")
print(f"🔍 Encontradas {len(images)} imágenes.")

# ---------------------------------------------------------
# 3. EXTRACCIÓN DE ESQUINAS
# ---------------------------------------------------------
fotos_validas = 0

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Buscar esquinas
    ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        fotos_validas += 1

        # Guardar imagen de comprobación
        cv2.drawChessboardCorners(img, pattern_size, corners2, ret)
        nombre_salida = os.path.join(CARPETA_SALIDA, os.path.basename(fname))
        cv2.imwrite(nombre_salida, img)
        print(f"✅ OK: {os.path.basename(fname)}")
    else:
        print(f"⚠️  FALLO: No se detectó patrón en {os.path.basename(fname)}")

print(f"\n✅ Se usaron {fotos_validas} de {len(images)} imágenes para calibrar.")

if fotos_validas == 0:
    print("❌ ERROR: No se detectó el tablero en ninguna foto. Revisa 'pattern_size'.")
    exit()

# ---------------------------------------------------------
# 4. CÁLCULO DE LA CALIBRACIÓN
# ---------------------------------------------------------
print("\n⏳ Calculando matrices (esto puede tardar unos segundos)...")
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

print("\n🎯 ¡Calibración completada!")
print("\nMatriz de cámara (camera_matrix.npy):")
print(mtx)
print("\nCoeficientes de distorsión (dist_coeffs.npy):")
print(dist)

np.save('camera_matrix.npy', mtx)
np.save('dist_coeffs.npy', dist)
print("\n💾 Archivos .npy guardados exitosamente.")

# ---------------------------------------------------------
# 5. CÁLCULO DEL ERROR DE REPROYECCIÓN
# ---------------------------------------------------------
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    mean_error += error

error_final = mean_error/len(objpoints)
print(f"\n📏 Error total de reproyección: {error_final:.4f} píxeles")

if error_final < 0.5:
    print("🌟 ¡RESULTADO EXCELENTE! Tu calibración es hiper-precisa.")
elif error_final < 1.0:
    print("👍 RESULTADO BUENO. Servirá perfectamente para Eurobot.")
else:
    print("⚠️ AVISO: Error por encima de 1.0. Intenta sacar fotos más nítidas o descartar las borrosas.")