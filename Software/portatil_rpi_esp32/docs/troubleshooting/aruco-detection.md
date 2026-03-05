# Troubleshooting - Detección ArUco (robot_localization)

## 🔍 Problemas Comunes de Detección ArUco

Este documento cubre problemas específicos de detección de marcadores ArUco en el sistema de localización basado en homografía.

**Sistema:** `robot_localization/field_localizer.py`  
**Referencias:** Ver `docs/guias/GUIA_ROBOT_LOCALIZATION.md`

---

## ⚠️ ArUcos No Son Detectados

### Síntomas
- El nodo `field_localizer` corre pero no detecta ningún ArUco
- Topic `/roborescue/robot_pose` no publica datos
- Logs muestran: "No ArUcos detected" o similar

### Causas Comunes

#### 1. **Problemas de Iluminación**

**Síntomas:** Los ArUcos son visibles pero el detector no los ve

**Soluciones:**
```bash
# Aumentar brillo de la cámara (si es cámara IP)
# O mejorar iluminación ambiental

# Verificar imagen cruda
ros2 run image_view image_view image:=/roborescue/zenital/image_raw

# Buscar marcadores negros/blancos claramente diferenciados
```

**Ajustes recomendados:**
- Coloca luces LED de 5000K a 6500K sobre el campo
- Evita sombras sobre los marcadores
- Asegura que el contraste blanco/negro es visible

---

#### 2. **Tamaño de ArUco Incorrecto**

**Síntomas:** ArUcos presentes pero no detectados

**Soluciones:**
```bash
# Verificar tamaño mínimo en field_localizer.py:
# min_marker_pixels = 50  # píxeles cuadrados mínimos

# Si ArUcos son demasiado pequeños:
# - Acerca la cámara más al campo
# - Usa lentes de mayor ángulo si es posible
# - Aumenta el tamaño de los ArUcos impresos

# Si ArUcos son detectados pero inconsistentes:
min_marker_pixels = 30  # Reduce este valor
```

**Tamaño recomendado:**
- Impresos en papel: 10cm × 10cm (mínimo 8cm)
- En campo de 300×200cm con cámara zenital

---

#### 3. **Diccionario ArUco Incorrecto**

**Síntomas:** Algunos ArUcos se detectan, otros no

**Soluciones:**
```python
# Verificar en field_localizer.py línea ~30:
# dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

# Opciones disponibles:
# - DICT_4X4_50      (0-49, pequeños)
# - DICT_5X5_100     (0-99)
# - DICT_6X6_250     (0-249, recomendado) ← ACTUAL
# - DICT_7X7_1000    (0-999, grandes)

# Cambiar diccionario si es necesario:
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
```

**IDs disponibles según diccionario:**
- ArUcos 20, 21, 22, 23 (esquinas) ✅ válidos en DICT_6X6_250
- ArUco 1 (robot) ✅ válido
- ArUco 36, 47 (cajas) ✅ válidos

---

#### 4. **Deformación Óptica (Distorsión de Cámara)**

**Síntomas:** ArUcos detectados pero posiciones incorrectas/saltando

**Soluciones:**
```bash
# Calibrar cámara (proceso de una sola vez)
# Usar patrón de tablero de ajedrez para calibrar distorsión

# Archivo de calibración debería estar en:
# config/camera_calibration.yaml

# Si no existe, crear uno:
# (Ver sección "Calibración de Cámara" abajo)

# En field_localizer.py, cargar calibración:
# with open('config/camera_calibration.yaml') as f:
#     calib = yaml.safe_load(f)
#     mtx = np.array(calib['camera_matrix']['data']).reshape(3, 3)
#     dist = np.array(calib['distortion_coefficients']['data'])
#     frame = cv2.undistort(frame, mtx, dist)
```

---

#### 5. **IDs de ArUco Incorrectos o Duplicados**

**Síntomas:** 
- Algunos ArUcos no se usan en posiciones
- Errores de homografía o outliers

**Soluciones:**
```python
# Verificar IDs en field_localizer.py (línea ~40):
REFERENCE_MARKERS = {
    20: (0, 0),       # Esquina superior izquierda ✅
    21: (300, 0),     # Esquina superior derecha ✅
    22: (0, 200),     # Esquina inferior izquierda ✅
    23: (300, 200)    # Esquina inferior derecha ✅
}

# Verificar que no hay duplicados
# Verificar que los IDs coinciden con los ArUcos impresos

# Para verificar qué ArUcos se detectan en tiempo real:
# (Ver sección "Verificación en Tiempo Real")
```

**Tabla de IDs configurados:**

| ID | Posición | Campo | Estado |
|----|----------|-------|--------|
| 20 | (0, 0) | Esquina TL | ✅ Referencia |
| 21 | (300, 0) | Esquina TR | ✅ Referencia |
| 22 | (0, 200) | Esquina BL | ✅ Referencia |
| 23 | (300, 200) | Esquina BR | ✅ Referencia |
| 1 | Variable | Robot | ✅ Móvil |
| 36 | Variable | Caja Azul | ✅ Móvil |
| 47 | Variable | Caja Amarilla | ✅ Móvil |

---

## 📍 Homografía No Converge / Posiciones Incorrectas

### Síntomas
- Posiciones aleatorias o muy saltando
- Pose2D publica valores inconsistentes
- Errores de reproyección altos

### Causas y Soluciones

#### 1. **Menos de 4 Marcadores de Referencia Visibles**

**Síntomas:** 
- Logs muestran "< 4 reference markers"
- Homografía se reutiliza indefinidamente

**Soluciones:**
```bash
# Reposicionar cámara para ver los 4 ArUcos de esquina
# Asegurar que toda el área del campo es visible

# Aumentar altura de cámara zenital si es necesario
# Reducir ángulo si hay perspectiva distorsionada

# Verificar en tiempo real:
ros2 topic echo /roborescue/robot_pose
# Si no cambia o muestra "NaN", problema de homografía
```

---

#### 2. **Homografía Mal Calculada (Outliers)**

**Síntomas:**
- Algunos puntos se detectan pero dan posiciones absurdas
- Posiciones "saltando" entre correctas e incorrectas

**Soluciones:**
```python
# En field_localizer.py, línea ~150 (homografía):
# Usar RANSAC para filtrar outliers

h, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

# Aumentar umbral si hay demasiados outliers:
h, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 10.0)

# Reducir si hay demasiados falsos positivos:
h, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 2.0)

# Verificar calidad de la homografía:
print(f"Homografía válida: {h is not None}")
print(f"Rango de valores: {np.min(h):.4f} a {np.max(h):.4f}")
```

---

#### 3. **Orden de Coordenadas Incorrecto**

**Síntomas:**
- Posiciones X e Y invertidas
- Robot reporta estar fuera del campo

**Verificación:**
```bash
# Mover robot a esquina conocida (ej: (0,0))
# Verificar que topic reporta cerca de (0, 0)

# Si reporta (300, 200) cuando debería ser (0, 0):
# Las esquinas están rotadas en el código

# Solución: Revisar mapping en field_localizer.py:
REFERENCE_MARKERS = {
    20: (0, 0),       # DEBE ser esquina superior izquierda
    21: (300, 0),     # DEBE ser esquina superior derecha
}
# Verificar que coincide con posición física real
```

---

## 🎥 Verificación en Tiempo Real

### Script para Diagnóstico

```python
# Crear archivo: verify_aruco.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArUcoVerifier(Node):
    def __init__(self):
        super().__init__('aruco_verifier')
        self.sub = self.create_subscription(Image, '/roborescue/zenital/image_raw', self.callback, 10)
        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.detector = cv2.aruco.ArucoDetector(self.dictionary)

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        corners, ids, rejected = self.detector.detectMarkers(frame)
        
        if ids is not None:
            self.get_logger().info(f"✅ Detectados {len(ids)} ArUcos: {ids.flatten().tolist()}")
            frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        else:
            self.get_logger().warn("❌ No ArUcos detected")
        
        # Mostrar frame
        cv2.imshow("ArUco Detection", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ArUcoVerifier()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

**Ejecutar:**
```bash
python3 verify_aruco.py
# Mostrará ventana con ArUcos detectados
# Logs mostrarán IDs detectados en tiempo real
```

---

## 🔧 Calibración de Cámara

### Problema
Si la cámara tiene distorsión óptica, las posiciones serán inexactas.

### Solución: Calibrar Cámara

#### 1. Capturar Imágenes de Calibración
```bash
# Imprimir patrón de tablero de ajedrez (8x6, 30mm cuadrados)
# https://docs.opencv.org/4.x/da/d0d/tutorial_camera_calibration_pattern.html

# Capturar 20-30 imágenes del patrón desde diferentes ángulos
ros2 run camera_calibration cameracalibrator.py --size 8x6 --square 0.03 image:=/roborescue/zenital/image_raw camera:=/camera

# Los archivos se guardarán en ~/.ros/
```

#### 2. Generar Archivo de Calibración
```bash
# El archivo generado contiene:
# - camera_matrix (matriz de intrínsecos)
# - distortion_coefficients (coeficientes de distorsión)

# Guardar en: src/robot_localization/config/camera_calibration.yaml
```

#### 3. Usar Calibración en field_localizer.py
```python
import yaml

with open('config/camera_calibration.yaml') as f:
    calib = yaml.safe_load(f)
    mtx = np.array(calib['camera_matrix']['data']).reshape(3, 3)
    dist = np.array(calib['distortion_coefficients']['data'])

# En el callback de imagen:
frame = cv2.undistort(frame, mtx, dist)  # Deshacer distorsión
```

---

## 🐛 Debugging Avanzado

### Activar Logs Detallados
```bash
# Nivel DEBUG
ros2 run robot_localization field_localizer --ros-args --log-level DEBUG

# Nivel WARN (menos verbose)
ros2 run robot_localization field_localizer --ros-args --log-level WARN
```

### Guardar Frames para Análisis Offline
```python
# En field_localizer.py, agregar después de detectar ArUcos:
if ids is not None:
    frame_with_markers = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
    cv2.imwrite(f'/tmp/frame_{frame_count}.jpg', frame_with_markers)
```

### Verificar Transformación de Coordenadas
```bash
# Crear pequeño test script
python3 -c "
import numpy as np
import cv2

# Puntos de prueba en imagen (píxeles)
img_pts = np.array([[50, 50], [250, 50], [50, 150], [250, 150]], dtype=np.float32)

# Puntos en campo (cm)
field_pts = np.array([[0, 0], [300, 0], [0, 200], [300, 200]], dtype=np.float32)

# Calcular homografía
H, _ = cv2.findHomography(img_pts, field_pts)

# Prueba: transformar punto (50, 50) en imagen
test_pt = np.array([[[50, 50]]], dtype=np.float32)
result = cv2.perspectiveTransform(test_pt, H)
print(f'(50, 50) img → {result[0, 0]} campo')  # Debería ser ~(0, 0)
"
```

---

## 📋 Checklist de Diagnóstico

- [ ] Cámara conectada y publicando en `/roborescue/zenital/image_raw`
- [ ] Iluminación suficiente (ver ArUcos claramente en imagen cruda)
- [ ] ArUcos impresos son del tamaño correcto (8-10cm)
- [ ] Los 4 ArUcos de referencia (20, 21, 22, 23) son visibles
- [ ] IDs de ArUcos coinciden con código en `field_localizer.py`
- [ ] Distancia de cámara al campo es apropiada (80-120cm)
- [ ] Ángulo de cámara es perpendicular al campo
- [ ] Topic `/roborescue/robot_pose` está recibiendo datos
- [ ] Posiciones están dentro del rango esperado (0-300cm en X, 0-200cm en Y)

---

## 🔗 Referencias

- **Código principal:** `src/robot_localization/robot_localization/field_localizer.py`
- **Guía de uso:** `docs/guias/GUIA_ROBOT_LOCALIZATION.md`
- **OpenCV ArUco:** https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
- **Calibración de cámara:** https://docs.opencv.org/4.x/d4/d94/tutorial_camera_calibration.html

---

**Última actualización:** 2026-03-05  
**Equipo:** RoboRescue
