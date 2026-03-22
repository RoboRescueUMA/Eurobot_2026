import cv2
import numpy as np
import glob
import os

carpeta_fotos = "fotos_calibracion"
carpeta_salida = "fotos_calibracion_detectadas"
os.makedirs(carpeta_salida, exist_ok=True)

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

        # Dibujar esquinas en la imagen
        cv2.drawChessboardCorners(img, pattern_size, corners2, ret)

        # Guardar la imagen con las esquinas dibujadas
        nombre_base = os.path.basename(fname)
        nombre_salida = os.path.join(carpeta_salida, nombre_base)
        cv2.imwrite(nombre_salida, img)
        print(f"✅ Guardada imagen detectada: {nombre_salida}")

        # Opcional: mostrar un momento para verlo en pantalla (comentar si quieres solo guardar)
        cv2.imshow('Calibración', img)
        cv2.waitKey(1000)   # o pon 0 si quieres pausar
    else:
        print(f"⚠️  No se detectó patrón en: {fname}")

cv2.destroyAllWindows()
print(f"✅ Se usaron {len(objpoints)} imágenes válidas para calibrar.")

# ... el resto del script (calibración y guardado) queda igual ...