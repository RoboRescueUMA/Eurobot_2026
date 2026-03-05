# Sistema de Localización Absoluta - Guía Rápida

## Introducción

El paquete `robot_localization` implementa **localización absoluta en el campo** usando **homografía con 4 ArUcos fijos** como referencia. Esto permite obtener las coordenadas reales (X, Y, θ) de cualquier objeto en el campo de forma robusta y precisa.

### Ventajas vs Sistema Anterior (Visión Relativa)

| Aspecto | Anterior (relativo) | Nuevo (absoluto) |
|--------|-------------------|-----------------|
| Posición | Relativa al robot | **Absoluta en el campo** |
| Referencia | Auto-calibración del robot | 4 ArUcos fijos (esquinas) |
| Robustez | Pierde tracking si no ve robot | Siempre localizado si ve ≥1 fijo |
| Navegación | Control hacia objetivos relativos | **Control hacia posiciones globales** |
| Inicialización | Auto-calibración inicial | Sin calibración (4 fijos = referencia) |

---

## Arquitectura

```
LAPTOP (WiFi)                    RPI4 (Ethernet)         ESP32 (USB)
├─ Camera Publisher ────►                               (Serial /dev/ttyUSB0)
├─ Field Localizer (NEW)─────►                          
│  ├─ Homografía                 Relay Node ────────────► Motor Control
│  └─ Poses ABSOLUTAS    /cmd_vel_laptop  /cmd_vel
```

### Hardware Requerido

- **Cámara**: IP Camera (móvil con app IPCamera) - Vista cenital
- **ArUcos fijos (esquinas)**:
  - ID=20: Esquina superior-izquierda (0, 0) cm
  - ID=21: Esquina superior-derecha (300, 0) cm
  - ID=22: Esquina inferior-izquierda (0, 200) cm
  - ID=23: Esquina inferior-derecha (300, 200) cm
- **ArUcos móviles**:
  - ID=1: Robot
  - ID=36: Caja azul
  - ID=47: Caja amarilla
- **Diccionario ArUco**: DICT_4X4_50
- **Tamaño de cada ArUco**: 5 cm (0.05 m)

---

## Sistema de Coordenadas

```
     (0,0) ◄─────────────────► (300,0)
     ArUco 20                ArUco 21
        ┌─────────────────────────┐
        │                         │
        │   CAMPO EUROBOT 2026    │  Y+
        │    300 cm × 200 cm      │  │
        │                         │  ▼
        │                         │
        └─────────────────────────┘
     ArUco 22                ArUco 23
    (0,200) ◄─────────────────► (300,200)
           X+ ►

Sistema: Origen = esquina superior-izquierda
         X+ = hacia derecha
         Y+ = hacia abajo
         θ = en grados (0° = apunta a +X)
```

---

## Uso

### 1. En LAPTOP (Localización + Camera)

```bash
cd ~/Desktop/GitHub/pruebas_eurobot
source install/setup.bash

# IMPORTANTE: Configurar ROS_DOMAIN_ID
export ROS_DOMAIN_ID=17

# Opción A: Usar launch file (recomendado)
ros2 launch robot_localization robot_localization.launch.py \
  camera_ip:=10.16.250.84:5000

# Opción B: Nodos individuales (alternativa)
ros2 run robot_localization camera_publisher --ros-args \
  -p video_url:=http://10.16.250.84:5000/video
ros2 run robot_localization field_localizer
```

**Esperado en logs:**
```
[field_localizer]: FieldLocalizer listo. Robot ID=1, Azul=36, Amarilla=47 | Fijos=[20, 21, 22, 23] | Campo 300x200 cm
[field_localizer]: Homografia actualizada con los 4 ArUcos fijos.
```

### 2. En RPI (Relay + micro-ROS agent)

Terminal 1 - micro-ROS agent:
```bash
cd ~/Desktop/GitHub/pruebas_eurobot
source install/setup.bash
export ROS_DOMAIN_ID=17

ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

Terminal 2 - Relay node:
```bash
cd ~/Desktop/GitHub/pruebas_eurobot
source install/setup.bash
export ROS_DOMAIN_ID=17

ros2 run rpi_relay cmd_vel_relay
```

### 3. Ver imagen debug (LAPTOP - Nueva Terminal)

```bash
export ROS_DOMAIN_ID=17

# RECOMENDADO: Video con detecciones ArUco y poses absolutas
ros2 run rqt_image_view rqt_image_view /roborescue/zenital/debug

# Alternativa: Video comprimido (menos lag por WiFi)
ros2 run rqt_image_view rqt_image_view /roborescue/zenital/image_raw/compressed
```

**En la imagen debug deberías ver:**
- Los 4 ArUcos fijos etiquetados como "F20", "F21", "F22", "F23"
- El robot etiquetado como "ROBOT (X cm, Y cm) angle deg" con flecha verde indicando orientación
- Las cajas etiquetadas como "AZUL" y "AMARILLA" con posiciones y ángulos
- Las esquinas transformadas del robot dibujadas con puntos para debug

---

## Verificar Funcionamiento

### Ver Topics Publicados

```bash
export ROS_DOMAIN_ID=17

# Ver lista de topics
ros2 topic list | grep roborescue

# Ver datos de posición del robot (ABSOLUTA en cm)
ros2 topic echo /roborescue/robot_pose
# Ejemplo:
# x: 150.0
# y: 100.0
# theta: 45.0  ← En grados (no radianes!)

# Ver posición de cajas
ros2 topic echo /roborescue/blue_box_pose
ros2 topic echo /roborescue/yellow_box_pose

# Ver frecuencia de publicación
ros2 topic hz /roborescue/robot_pose
```

### Verificar Homografía

Si los 4 ArUcos fijos NO están visibles:
```
[field_localizer]: Esperando 4 ArUcos fijos (IDs 20,21,22,23)...
```

Si la homografía se recalcula exitosamente:
```
[field_localizer]: Homografia actualizada con los 4 ArUcos fijos.
```

---

## Colocación Correcta de ArUcos

### Los 4 ArUcos Fijos (Esquinas del Campo)

**IMPORTANTE:** Los 4 ArUcos fijos deben colocarse en las **esquinas del campo** de competición:

```
┌──────────────────────────────────┐
│ (0,0) ID=20      (300,0) ID=21   │
│                                  │
│  CAMPO DE EUROBOT 2026           │
│  300 cm × 200 cm                 │
│                                  │
│ (0,200) ID=22  (300,200) ID=23   │
└──────────────────────────────────┘
```

**Verificación en imagen debug:**
- Deberías ver un rectángulo aproximado con las 4 esquinas marcadas
- Los ArUcos fijos se etiquetan con "F" (ej: F20, F21, F22, F23)

### El ArUco del Robot (ID=1)

**Colocación:** Montado sobre el robot, **esquina 0 apuntando hacia adelante**

```
        Adelante del robot
              ↑
              │
    ┌─────────────────┐
    │    ArUco ID=1   │
    │                 │
    │   ◄─────────┐   │
    │   │         │   │ ← Esquina 0 (círculo rojo en debug)
    │   │    1    │   │    debe apuntar ADELANTE
    │   └─────────┘   │
    │                 │
    └─────────────────┘
```

**Verificación:**
- En la imagen debug, el círculo rojo del robot debe estar en la parte delantera
- La flecha verde debe apuntar en la dirección hacia la que el robot avanza

---

## Topics

### Publicados por robot_localization

- `/roborescue/zenital/image_raw` - Video de cámara sin comprimir (Image)
- `/roborescue/zenital/image_raw/compressed` - Video comprimido (CompressedImage) - **Recomendado para WiFi**
- `/roborescue/zenital/debug` - Video con anotaciones ArUco y poses (Image)
- **`/roborescue/robot_pose`** - **Posición ABSOLUTA del robot** (Pose2D: x en cm, y en cm, theta en grados)
- **`/roborescue/blue_box_pose`** - **Posición ABSOLUTA de caja azul** (Pose2D)
- **`/roborescue/yellow_box_pose`** - **Posición ABSOLUTA de caja amarilla** (Pose2D)

### Publicados por rpi_relay

- `/roborescue/cmd_vel` - Comandos de velocidad para ESP32 (Twist)

### Subscritos por robot_localization

- `/roborescue/zenital/image_raw` - Stream de cámara

### Subscritos por ESP32 (vía micro-ROS)

- `/roborescue/cmd_vel` - Velocidades (Twist: linear.x, linear.y, angular.z)

---

## Parámetros Ajustables

En `/home/maki/Desktop/GitHub/pruebas_eurobot/src/robot_localization/config/robot_localization.yaml`:

```yaml
field_localizer:
  ros__parameters:
    # IDs de los objetos móviles
    robot_id: 1              # ID del ArUco del robot
    blue_box_id: 36          # ID de la caja azul
    yellow_box_id: 47        # ID de la caja amarilla

    # IDs de los ArUcos fijos (debe coincidir con tu colocación)
    fixed_ids: [20, 21, 22, 23]  # [sup_izq, sup_der, inf_izq, inf_der]

    # Dimensiones del campo
    field_width_cm: 300.0    # Ancho (X)
    field_height_cm: 200.0   # Alto (Y)

    # Actualización de homografía
    homography_update_every_n_frames: 30  # Cada 30 frames @ 10 Hz = cada 3 seg
```

---

## Troubleshooting

### Problema: "Esperando 4 ArUcos fijos (IDs 20,21,22,23)..."

**Causa:** La cámara no ve los 4 ArUcos fijos simultáneamente.

**Solución:**
1. Verificar que los 4 ArUcos están colocados en las esquinas y son visibles
2. Verificar que el diccionario es DICT_4X4_50
3. Verificar que los IDs coinciden con los parámetros en `robot_localization.yaml`
4. Aumentar luz o mejorar contraste de los ArUcos
5. Verificar que la cámara tiene una vista clara del campo

### Problema: Posiciones erráticas o que saltan

**Causa:** Jitter en la detección de ArUcos o homografía inestable.

**Solución:**
1. Reducir `homography_update_every_n_frames` (ejemplo: 10 en lugar de 30)
2. Implementar promedio móvil de poses (próxima mejora planificada)
3. Verificar iluminación uniforme

### Problema: El robot se ve pero no las cajas

**Solución:**
1. Verificar IDs: robot=1, azul=36, amarilla=47
2. Cambiar IDs en `robot_localization.yaml` si es necesario
3. Colocar las cajas en el campo dentro del área visible

### Problema: Ángulos incorrectos (theta)

**Causa:** La esquina 0 del ArUco no está correctamente orientada.

**Solución:**
1. Ver la imagen debug: esquina 0 debe ser el círculo rojo
2. Rotar el ArUco del robot para que la esquina 0 apunte hacia adelante
3. Para las cajas, la orientación se auto-calcula

---

## Próximos Pasos

- [ ] Implementar `aruco_navigator` - navegación con poses absolutas
- [ ] Añadir filtrado/promedio móvil para reducir jitter
- [ ] Validar coordenadas en competición real
- [ ] Evitación de obstáculos (si el reglamento lo permite)
- [ ] Estrategia de recaudación automática de objetivos

---

## Referencias

- OpenCV Homography: https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#gaf592a61d370d2e6c5bea1a2f2c88f0ff
- ArUco Detection: https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
- ROS2 Humble: https://docs.ros.org/en/humble/

---

**Versión:** 1.0  
**Fecha:** Marzo 2026  
**Sistema:** robot_localization - Localización Absoluta con Homografía
