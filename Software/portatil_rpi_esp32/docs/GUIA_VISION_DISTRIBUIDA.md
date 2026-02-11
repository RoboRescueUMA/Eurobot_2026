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

# Opción A: Usar launch file (recomendado)
ros2 launch laptop_vision laptop_vision.launch.py camera_ip:=192.168.100.122:5000 target:=blue_box

# Opción B: Nodos individuales
ros2 run laptop_vision camera_publisher --ros-args -p video_url:=http://192.168.100.122:5000/video
ros2 run laptop_vision aruco_detector
ros2 run laptop_vision aruco_navigator --ros-args -p target:=blue_box
```

### 2. En RPI (Relay + micro-ROS agent)

Terminal 1 - micro-ROS agent:
```bash
cd ~/Desktop/ros2_pi_esp
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

Terminal 2 - Relay node:
```bash
cd ~/Desktop/ros2_pi_esp
source install/setup.bash
ros2 run rpi_relay cmd_vel_relay
```

### 3. Ver imagen debug (LAPTOP)

```bash
# Opción A: Video con detecciones ArUco y anotaciones (recomendado)
ros2 run rqt_image_view rqt_image_view /robot1/zenital/debug

# Opción B: Video original sin comprimir (más lag por WiFi)
ros2 run rqt_image_view rqt_image_view /robot1/zenital/image_raw

# Opción C: Video comprimido (mejor rendimiento en WiFi)
ros2 run rqt_image_view rqt_image_view /robot1/zenital/image_raw/compressed
```

**Recomendación:** Usa `/compressed` si experimentas lag en el video.

## Topics

### Laptop publica:
- `/robot1/zenital/image_raw` - Video de cámara sin comprimir (Image)
- `/robot1/zenital/image_raw/compressed` - Video comprimido (CompressedImage) - **Recomendado para WiFi**
- `/robot1/zenital/debug` - Video con anotaciones ArUco (Image)
- `/robot1/robot_pos` - Posición del robot (siempre 0,0)
- `/robot1/blue_box_pos` - Posición de caja azul
- `/robot1/yellow_box_pos` - Posición de caja amarilla  
- `/robot1/cmd_vel_laptop` - Comandos de velocidad

### RPI relay:
- Suscribe: `/robot1/cmd_vel_laptop`
- Publica: `/cmd_vel` (para ESP32)

### ESP32 suscribe:
- `/cmd_vel`

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

| Problema | Solución |
|----------|----------|
| No detecta ArUcos | Verificar iluminación, imprimir ArUcos más grandes (mínimo 5cm) |
| Lag en video | Revisar que QoS=1 esté activo, reducir `publish_rate` |
| Robot no se mueve | Verificar que relay esté corriendo en RPI y micro-ROS agent activo |
| Movimiento errático | Ajustar `linear_p_gain` y `angular_p_gain` (bajar valores) |
| Cámara no conecta | Verificar IP, que IPCamera app esté activa, laptop y móvil en misma WiFi |

## Competencia (ROS_DOMAIN_ID)

Para evitar conflictos con otros equipos:

```bash
export ROS_DOMAIN_ID=42  # Cambiar número único
ros2 launch laptop_vision laptop_vision.launch.py domain_id:=42
```

## Próximos Pasos

- [ ] Calibrar ganancias PID con robot real
- [ ] Añadir evitación de obstáculos
- [ ] Implementar strafing con Mecanum wheels (cmd.linear.y)
- [ ] Planificación de trayectorias para múltiples cajas
