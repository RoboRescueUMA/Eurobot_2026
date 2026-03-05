# Sistema de Visión Distribuida - Guía Rápida

## Arquitectura

```
LAPTOP (WiFi)              RPI4 (Ethernet)              ESP32 (USB)
├─ Camera Publisher ────►  (WiFi)                       (Serial /dev/ttyUSB0)
├─ ArUco Detector   ────►                               
└─ Navigator        ────►  Relay Node ────────────────► Motor Control
   /robot1/cmd_vel_laptop  /cmd_vel
```

## Hardware

- **Cámara**: IP Camera (móvil con app IPCamera) - Vista cenital
- **ArUcos**: Robot ID=1, Caja Azul ID=36, Caja Amarilla ID=47
- **Tamaño ArUco**: 5cm (0.05m)
- **ESP32**: Conectado por USB a RPI (/dev/ttyUSB0)

## Uso

### 1. En LAPTOP (Procesamiento de visión)

```bash
cd ~/Desktop/GitHub/pruebas_eurobot
source install/setup.bash

# IMPORTANTE: Configurar ROS_DOMAIN_ID
export ROS_DOMAIN_ID=17

# Opción A: Usar launch file (recomendado)
ros2 launch laptop_vision laptop_vision.launch.py camera_ip:=10.16.250.84:5000 target:=blue_box

# Opción B: Nodos individuales
ros2 run laptop_vision camera_publisher --ros-args -p video_url:=http://10.16.250.84:5000/video
ros2 run laptop_vision aruco_detector
ros2 run laptop_vision aruco_navigator --ros-args -p target:=blue_box
```

### 2. En RPI (Relay + micro-ROS agent)

Terminal 1 - micro-ROS agent:
```bash
cd ~/pruebas_eurobot
source install/setup.bash

# IMPORTANTE: Configurar ROS_DOMAIN_ID
export ROS_DOMAIN_ID=17

ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

Terminal 2 - Relay node:
```bash
cd ~/pruebas_eurobot
source install/setup.bash

# IMPORTANTE: Configurar ROS_DOMAIN_ID
export ROS_DOMAIN_ID=17

ros2 run rpi_relay cmd_vel_relay
```

### 3. Ver imagen debug (LAPTOP)

```bash
# IMPORTANTE: Configurar ROS_DOMAIN_ID
export ROS_DOMAIN_ID=17

# Opción A: Video con detecciones ArUco y anotaciones (recomendado)
ros2 run rqt_image_view rqt_image_view /roborescue/zenital/debug

# Opción B: Video original sin comprimir (más lag por WiFi)
ros2 run rqt_image_view rqt_image_view /roborescue/zenital/image_raw

# Opción C: Video comprimido (mejor rendimiento en WiFi)
ros2 run rqt_image_view rqt_image_view /roborescue/zenital/image_raw/compressed
```

**Recomendación:** Usa `/compressed` si experimentas lag en el video.

## Topics

### Laptop publica:
- `/roborescue/zenital/image_raw` - Video de cámara sin comprimir (Image)
- `/roborescue/zenital/image_raw/compressed` - Video comprimido (CompressedImage) - **Recomendado para WiFi**
- `/roborescue/zenital/debug` - Video con anotaciones ArUco (Image)
- `/roborescue/robot_pose` - Posición del robot (PoseStamped)
- `/roborescue/blue_box_pose` - Posición de caja azul (PoseStamped)
- `/roborescue/yellow_box_pose` - Posición de caja amarilla (PoseStamped)
- `/roborescue/cmd_vel_laptop` - Comandos de velocidad (Twist)

### RPI relay:
- Suscribe: `/roborescue/cmd_vel_laptop`
- Publica: `/roborescue/cmd_vel` (para ESP32)

### ESP32 suscribe:
- `/roborescue/cmd_vel`

### ESP32 publica:
- `/roborescue/encoder_velocities` - Velocidades de encoders [FL, FR, RL, RR] en RPM (Float32MultiArray)

## Parámetros Ajustables

En `laptop_vision/config/camera.yaml` o por línea de comandos:

```bash
ros2 run laptop_vision aruco_navigator --ros-args \
  -p target:=yellow_box \
  -p max_linear_speed:=0.5 \
  -p max_angular_speed:=1.5 \
  -p linear_p_gain:=0.8 \
  -p angular_p_gain:=2.0 \
  -p goal_tolerance:=0.05
```

## Troubleshooting

Ver problemas comunes y soluciones en: `docs/troubleshooting/`

| Problema | Archivo de referencia |
|----------|----------------------|
| No detecta ArUcos | `docs/troubleshooting/aruco-no-detecta.md` |
| Robot no se mueve | `docs/troubleshooting/robot-no-se-mueve.md` |
| micro-ROS no conecta | `docs/troubleshooting/micro-ros-device-busy.md` |

## Configuración Permanente ROS_DOMAIN_ID

Para evitar tener que exportar ROS_DOMAIN_ID=17 en cada terminal:

```bash
# Añadir a ~/.bashrc (tanto en laptop como en RPI)
echo "export ROS_DOMAIN_ID=17" >> ~/.bashrc
source ~/.bashrc
```

## Competencia (Cambiar ROS_DOMAIN_ID)

Para evitar conflictos con otros equipos en competición:

```bash
# Cambiar número único por equipo (ej. 42)
export ROS_DOMAIN_ID=42

# Actualizar en ~/.bashrc
nano ~/.bashrc
# Cambiar: export ROS_DOMAIN_ID=17 → export ROS_DOMAIN_ID=42
```

## Próximos Pasos

- [ ] Calibrar ganancias PID con robot real
- [ ] Añadir evitación de obstáculos
- [ ] Implementar strafing con Mecanum wheels (cmd.linear.y)
- [ ] Planificación de trayectorias para múltiples cajas
