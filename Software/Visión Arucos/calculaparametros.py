import cv2
import numpy as np
import glob
import os

carpeta_fotos = "fotos_calibracion"
pattern_size = (7, 10)
square_size = 0.035
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
objp *= square_size

objpoints = []
imgpoints = []

images = glob.glob(os.path.join(carpeta_fotos, '*.jpg')) + \
         glob.glob(os.path.join(carpeta_fotos, '*.png'))

print(f"📂 Buscando imágenes en: {carpeta_fotos}")
print(f"🔍 Encontradas {len(images)} imágenes.")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
    else:
        print(f"⚠️  No se detectó patrón en: {fname}")

print(f"✅ Se usaron {len(objpoints)} imágenes válidas para calibrar.")

if len(objpoints) == 0:
    exit()

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

print("\n🎯 Calibración completada.")
print("Matriz de cámara (intrínsecos):")
print(mtx)
print("\nCoeficientes de distorsión:")
print(dist)

np.save('camera_matrix.npy', mtx)
np.save('dist_coeffs.npy', dist)

mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    mean_error += error
print(f"\n📏 Error total de reproyección: {mean_error/len(objpoints):.4f} píxeles (ideal < 0.5)")