# robot_localization Package

## 📍 Descripción

El paquete **robot_localization** implementa un sistema de **localización absoluta y navegación autónoma** del robot en el campo usando:

- **4 marcadores ArUco fijos** en las esquinas del campo (referencia de posición)
- **Homografía 2D** para convertir coordenadas de píxeles de cámara a coordenadas del campo
- **Detección de ArUcos móviles** para rastrear el robot y objetos

**Sistema:** Cámara zenital → Detección ArUco → Homografía → Pose absoluta en el campo

---

## 🏗️ Estructura del Paquete

```
robot_localization/
├── robot_localization/          # Código fuente
│   ├── __init__.py
│   ├── camera_publisher.py      # Nodo: captura y publica frames de cámara IP
│   ├── field_localizer.py       # Nodo: detecta ArUcos y calcula homografía
│   └── aruco_navigator.py       # Nodo: (futuro) navega hacia coordenadas
├── launch/
│   └── robot_localization.launch.py  # Launch file para ambos nodos
├── config/
│   └── robot_localization.yaml  # Configuración: IDs ArUco, dimensiones campo
├── package.xml                  # Metadata del paquete
├── setup.py                     # Configuración Python
└── README.md                    # Este archivo
```

---

## 🚀 Compilación

```bash
# En la raíz del workspace
cd ~/Desktop/GitHub/pruebas_eurobot

# Compilar solo este paquete
colcon build --packages-select robot_localization

# Compilar y symlink (modo desarrollo)
colcon build --packages-select robot_localization --symlink-install

# Fuente el workspace
source install/setup.bash
```

---

## 🎯 Nodos Principales

### 1. **camera_publisher.py** — Publicación de Frames

**Propósito:** Capturar frames de una cámara IP y publicarlos como tópico ROS

**Tópico publicado:**
- `/roborescue/zenital/image_raw` — Frames crudos (sensor_msgs/Image)

**Parámetros (config/robot_localization.yaml):**
- `camera_ip` — IP de la cámara (ej: "192.168.1.100")
- `camera_port` — Puerto de la cámara (por defecto: 5000)
- `frame_rate` — FPS deseado (por defecto: 10 Hz)

**Ejecución:**
```bash
ros2 run robot_localization camera_publisher
```

---

### 2. **field_localizer.py** — Detección y Localización

**Propósito:** Detectar marcadores ArUco, calcular homografía y publicar posiciones absolutas

**Entrada:**
- `/roborescue/zenital/image_raw` — Frames de cámara

**Tópicos publicados:**
- `/roborescue/robot_pose` — Posición del robot: `Pose2D` (x, y en cm; theta en °)
- `/roborescue/blue_box_pose` — Posición caja azul (ArUco 36)
- `/roborescue/yellow_box_pose` — Posición caja amarilla (ArUco 47)
- `/roborescue/field_debug` — Debug: imagen con ArUcos dibujados

**Parámetros (config/robot_localization.yaml):**
```yaml
reference_markers:
  20: [0, 0]           # ArUco 20 → esquina (0, 0) del campo
  21: [300, 0]         # ArUco 21 → esquina (300, 0)
  22: [0, 200]         # ArUco 22 → esquina (0, 200)
  23: [300, 200]       # ArUco 23 → esquina (300, 200)

field_width_cm: 300    # Ancho del campo en cm
field_height_cm: 200   # Alto del campo en cm
```

**Ejecución:**
```bash
ros2 run robot_localization field_localizer
```

---

### 3. **aruco_navigator.py** — Navegación Autónoma (Futuro)

**Estado:** Todavía no implementado

**Propósito (cuando se implemente):**
- Suscribirse a waypoints o destinos
- Usar `field_localizer` para obtener posición actual
- Calcular comandos de movimiento para llegar al destino
- Publicar a `/roborescue/cmd_vel`

---

## 🎬 Lanzar Todo de Una Vez

```bash
# Opción 1: Con launch file (recomendado)
ros2 launch robot_localization robot_localization.launch.py

# Opción 2: Nodos individuales en diferentes terminales
# Terminal 1:
ros2 run robot_localization camera_publisher

# Terminal 2:
ros2 run robot_localization field_localizer
```

---

## 📊 Sistema de Coordenadas

```
(0, 0) -------- (300, 0)    Arriba (Y=0)
  │                │
  │   CAMPO        │
  │   300x200cm    │
  │                │
  └──────────────┘
(0, 200) ---- (300, 200)    Abajo (Y=200)

Izquierda          Derecha
(X=0)              (X=300)
```

**Unidades:**
- X, Y: centímetros (cm)
- θ (theta): grados (°), en rango [0°, 360°)
  - 0° = derecha (+X)
  - 90° = abajo (+Y)
  - 180° = izquierda (-X)
  - 270° = arriba (-Y)

---

## 🏷️ Marcadores ArUco

### Fijos (Referencia de Posición)

| ID | Posición (cm) | Ubicación |
|----|--|--|
| **20** | (0, 0) | Esquina superior izquierda |
| **21** | (300, 0) | Esquina superior derecha |
| **22** | (0, 200) | Esquina inferior izquierda |
| **23** | (300, 200) | Esquina inferior derecha |

### Móviles (Objetos Rastreados)

| ID | Objeto | Tópico publicado |
|----|--------|--|
| **1** | Robot | `/roborescue/robot_pose` |
| **36** | Caja azul | `/roborescue/blue_box_pose` |
| **47** | Caja amarilla | `/roborescue/yellow_box_pose` |

---

## 📋 Requisitos

### Hardware
- Cámara IP con streaming MJPEG o H.264
- Conexión de red (Ethernet o Wi-Fi) desde RPI a cámara
- ArUcos impresos (6x6, diccionario DICT_6X6_250) de 8-10cm

### Software
- ROS2 Humble
- Python 3.10+
- OpenCV 4.5+ con módulo ArUco
- geometry_msgs (incluido en ROS2)

### Instalación de Dependencias

```bash
# Ubuntu/Debian
sudo apt update
sudo apt install -y python3-opencv python3-colcon-common-extensions

# ROS2 packages
sudo apt install -y ros-humble-cv-bridge ros-humble-image-transport
```

---

## 🔧 Configuración

### Archivo Principal: `config/robot_localization.yaml`

```yaml
# Identificadores de marcadores ArUco
reference_markers:
  20: [0, 0]
  21: [300, 0]
  22: [0, 200]
  23: [300, 200]

mobile_markers:
  1: "robot"
  36: "blue_box"
  47: "yellow_box"

# Dimensiones del campo (cm)
field_width_cm: 300
field_height_cm: 200

# Parámetros de cámara
camera_ip: "192.168.1.100"
camera_port: 5000
camera_topic_base: "zenital"

# Parámetros de detección
aruco_dict: "DICT_6X6_250"
min_marker_pixels: 50

# Parámetros de homografía
homography_update_frames: 30  # Recalcular H cada N frames
homography_ransac_threshold: 5.0
```

### Cambiar Configuración

**Opción A: Editar archivo YAML**
```bash
# Editar: src/robot_localization/config/robot_localization.yaml
# Cambiar valores, guardar, y relanzar nodos
```

**Opción B: Parámetros por línea de comandos**
```bash
ros2 run robot_localization field_localizer \
    --ros-args \
    -p field_width_cm:=400 \
    -p field_height_cm:=300
```

---

## 📊 Topología de Nodos y Topics

```
┌─────────────────────────────────────────────────────┐
│ Camera IP (Hardware)                                │
└─────────────────────┬───────────────────────────────┘
                      │ HTTP Stream
┌─────────────────────▼───────────────────────────────┐
│ camera_publisher (nodo)                             │
│ - Lee stream HTTP de cámara IP                      │
│ - Convierte a ROS Image message                     │
└─────────────────────┬───────────────────────────────┘
                      │
            /roborescue/zenital/
            image_raw (sensor_msgs/Image)
                      │
┌─────────────────────▼───────────────────────────────┐
│ field_localizer (nodo)                              │
│ - Detecta ArUcos en imagen                          │
│ - Calcula homografía de 4 referencias               │
│ - Transforma coordenadas a campo                    │
└─┬───────────┬──────────────────────┬────────────────┘
  │           │                      │
  │           │                      │
  ▼           ▼                      ▼
/roborescue/ /roborescue/         /roborescue/
robot_pose   blue_box_pose        yellow_box_pose
(Pose2D)     (Pose2D)             (Pose2D)
```

---

## 🧪 Testing y Debugging

### Verificar que Nodos Corren

```bash
# Ver nodos activos
ros2 node list
# Debería mostrar:
# /camera_publisher
# /field_localizer

# Ver topics
ros2 topic list
# Debería mostrar:
# /roborescue/zenital/image_raw
# /roborescue/robot_pose
# /roborescue/blue_box_pose
# /roborescue/yellow_box_pose
```

### Monitorear Poses en Tiempo Real

```bash
# Robot
ros2 topic echo /roborescue/robot_pose

# Caja azul
ros2 topic echo /roborescue/blue_box_pose

# Caja amarilla
ros2 topic echo /roborescue/yellow_box_pose
```

### Ver Imagen con ArUcos Detectados (Debug)

```bash
# En terminal con ROS sourced:
ros2 run image_view image_view image:=/roborescue/zenital/image_raw

# O si tienes debug image:
ros2 run image_view image_view image:=/roborescue/field_debug
```

---

## 🐛 Troubleshooting

Para problemas comunes, ver:
- **ArUcos no detectados:** `docs/troubleshooting/aruco-detection.md`
- **Posiciones incorrectas:** `docs/troubleshooting/aruco-detection.md` → sección "Homografía"
- **Cámara no conecta:** `docs/guias/GUIA_ROBOT_LOCALIZATION.md` → sección "Troubleshooting"

---

## 📚 Documentación Relacionada

- **Guía de Uso Completa:** `docs/guias/GUIA_ROBOT_LOCALIZATION.md`
- **Troubleshooting ArUco:** `docs/troubleshooting/aruco-detection.md`
- **Arquitectura del Sistema:** `docs/architecture/ARQUITECTURA_SISTEMA.md`
- **OpenCV ArUco Oficial:** https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html

---

## 🔗 Código Principal

- `camera_publisher.py` — Lee cámara IP y publica frames
- `field_localizer.py` — Detecta ArUcos y calcula homografía (PRINCIPAL)
- `aruco_navigator.py` — Placeholder para navegación (futuro)

---

## 📦 Instalación Rápida

```bash
# 1. Compilar
colcon build --packages-select robot_localization

# 2. Fuente
source install/setup.bash

# 3. Editar configuración si es necesario
nano src/robot_localization/config/robot_localization.yaml

# 4. Lanzar
ros2 launch robot_localization robot_localization.launch.py
```

---

## 📝 Notas de Desarrollo

- **Diccionario ArUco:** Usa DICT_6X6_250 (IDs 0-249)
- **Frecuencia:** 10 Hz (cámara y detección)
- **Latencia:** ~100-150 ms (captura + procesamiento)
- **Precisión:** ±2-3 cm típicamente (depende de calibración de cámara)

---

**Última actualización:** 2026-03-05  
**Mantenedor:** Team RoboRescue
