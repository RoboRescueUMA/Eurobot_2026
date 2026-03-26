# Arquitectura del Sistema - Robot Eurobot 2026

## Visión General

El sistema está diseñado con una **arquitectura distribuida** de tres niveles que separa las responsabilidades por capacidad computacional y función:

1. **Portátil** - Procesamiento de visión (localización + navegación)
2. **Raspberry Pi 4** - Relay de comandos y comunicación
3. **ESP32** - Control de bajo nivel y actuadores

### Sistema Actual: Localización Absoluta con Homografía

El sistema implementa **localización absoluta en el campo** mediante:
- 4 ArUcos fijos en las esquinas del campo como referencia (IDs 20-23)
- Cálculo de homografía píxeles ↔ coordenadas reales del campo
- Transformación de posiciones/orientaciones a sistema global (X, Y en cm)

---

## Diagrama de Arquitectura General

```
╔════════════════════════════════════════════════════════════════════╗
║              ARQUITECTURA DEL SISTEMA (CON HOMOGRAFÍA)             ║
╚════════════════════════════════════════════════════════════════════╝

┌──────────────────────────────────────────────────────────────────────┐
│                      NIVEL DE PERCEPCIÓN                             │
│                        (PORTÁTIL)                                    │
├──────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌─────────────┐     ┌──────────────────┐       ┌─────────────────┐  │
│  │  Cámara IP  │────►│  Nodo OpenCV     │──────►│ Nodo Localiza   │  │
│  │  (Ethernet/ │     │  - Captura       │       │ ArUco           │  │
│  │   WiFi)     │     │  - Preproceso    │       │ - Homografía    │  │
│  └─────────────┘     └──────────────────┘       │ - Poses ABS     │  │
│                                │                └─────────────────┘  │
│                                ▼                        │            │
│                      /zenital/image_raw      /robot_pose (ABS)       │
│                                              /blue_box_pose (ABS)    │
│                                              /yellow_box_pose (ABS)  │
│                                                                      │
│  CLAVE: Coordenadas ABSOLUTAS del campo (cm)                         │
│  - Origen: esquina sup-izq (ArUco 20)                                │
│  - 4 ArUcos fijos (20,21,22,23) en esquinas                          │
│  - Homografía recalculada cada 30 frames si están visibles           │
└────────────────────────────────┬───────────────────┬─────────────────┘
                                 │     ROS2 WiFi     │
                                 │    DDS Network    │
┌────────────────────────────────┴───────────────────┴────────────────┐
│              NIVEL DE DECISIÓN Y CONTROL                            │
│                   (RASPBERRY PI 4)                                  │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌───────────────────────────────────────────────────────────────┐  │
│  │        micro-ROS Agent                                        │  │
│  │  - Puente ROS2 ↔ micro-ROS                                    │  │
│  │  - Serial/WiFi con ESP32                                      │  │
│  │  - Relayea /cmd_vel_laptop → /cmd_vel                         │  │
│  └────────────────────────┬──────────────────────────────────────┘  │
│                           │                                         │
│                           ▼                                         │
│                    /cmd_vel (Twist)                                 │
└─────────────────────────────┼───────────────────────────────────────┘
                              │   Serial/WiFi
                              │   micro-ROS
┌─────────────────────────────┴───────────────────────────────────────┐
│                    NIVEL DE ACTUACIÓN                               │
│                        (ESP32)                                      │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌────────────────┐    ┌────────────────────┐   ┌───────────────┐   │
│  │ Nodo micro-ROS │───►│ Cinemática Inversa │──►│ Control PWM   │   │
│  │ - Sub /cmd_vel │    │ Mecanum (X-Drive)  │   │ - 4 motores   │   │
│  │ - Watchdog     │    │ - Cálculo ruedas   │   │ - 2 drivers H │   │
│  └────────────────┘    └────────────────────┘   └───────┬───────┘   │
│                                                         │           │
│                                                         ▼           │
│                                          ┌──────────────────────┐   │
│                                          │   Hardware Físico    │   │
│                                          │ - 4 Motores Mecanum  │   │
│                                          │ - 2 Puentes H        │   │
│                                          │ - Encoders           │   │
│                                          └──────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Flujo de Datos Detallado

### 1. Percepción → Decisión

```
[Cámara IP]
    │
    ▼ (stream RTSP/HTTP)
[Portátil: Nodo captura]
    │
    ▼ (OpenCV processing)
[Portátil: Detector ArUco/Objetos]
    │
    ▼ (ROS2 topic: /detections)
[RPI4: Fusión sensorial]
    │
    ├─► Posición robot (x, y, θ)
    ├─► Objetos detectados
    └─► Mapa de obstáculos
```

### 2. Decisión → Control

```
[RPI4: Fusión Sensorial]
    │
    ▼ (Estado actual del robot)
[RPI4: Planificador]
    │
    ├─► Estrategia de juego
    ├─► Path Planning (A*, DWA, etc.)
    └─► Waypoints objetivo
    │
    ▼
[RPI4: Controlador de navegación]
    │
    ├─► Control PID
    ├─► Cálculo velocidades (vx, vy, wz)
    └─► Publicar /cmd_vel
```

### 3. Control → Actuación

```
[RPI4: Topic /cmd_vel]
    │
    ▼ (micro-ROS bridge)
[RPI4: micro-ROS Agent]
    │
    ▼ (Serial/WiFi)
[ESP32: micro-ROS Client]
    │
    ▼ (Callback subscription)
[ESP32: Cinemática Mecanum]
    │
    ├─► Calcular velocidad FL
    ├─► Calcular velocidad FR
    ├─► Calcular velocidad RL
    └─► Calcular velocidad RR
    │
    ▼
[ESP32: Control PWM]
    │
    ├─► Motor FL (PWM + DIR)
    ├─► Motor FR (PWM + DIR)
    ├─► Motor RL (PWM + DIR)
    └─► Motor RR (PWM + DIR)
    │
    ▼
[Motores físicos] → Robot se mueve
```

---

## Componentes de Software

### Portátil (Ubuntu 22.04 + ROS2 Humble)

#### Paquete: `robot_localization` (NUEVO)

**Nodos:**
- `camera_publisher` - Captura de stream de cámara IP
- `field_localizer` - Detección de ArUcos + Homografía → Poses absolutas
- `aruco_navigator` (futuro) - Navegación hacia objetivos en coordenadas del campo

**Topics publicados:**
- `/roborescue/zenital/image_raw` (sensor_msgs/Image)
- `/roborescue/zenital/image_raw/compressed` (CompressedImage) - **Recomendado para WiFi**
- `/roborescue/zenital/debug` (sensor_msgs/Image) - Con anotaciones ArUco
- `/roborescue/robot_pose` (geometry_msgs/Pose2D) - **Posición ABSOLUTA en cm + theta en grados**
- `/roborescue/blue_box_pose` (geometry_msgs/Pose2D) - **Posición ABSOLUTA**
- `/roborescue/yellow_box_pose` (geometry_msgs/Pose2D) - **Posición ABSOLUTA**

**Parámetros:**
- `robot_id` - ID del ArUco del robot (default: 1)
- `blue_box_id` - ID de caja azul (default: 36)
- `yellow_box_id` - ID de caja amarilla (default: 47)
- `fixed_ids` - IDs de los 4 ArUcos fijos (default: [20, 21, 22, 23])
- `field_width_cm` - Ancho del campo (default: 300)
- `field_height_cm` - Alto del campo (default: 200)
- `homography_update_every_n_frames` - Frecuencia de recalculación (default: 30)

**Dependencias:**
- OpenCV (cv2) - Para detección de ArUcos y transformaciones perspectivas
- cv_bridge
- NumPy

---

#### Paquete: `laptop_vision` (ANTERIOR - Referencia)

**Nodos:** (Sistema antiguo de visión relativa)
- `camera_publisher` - Captura de cámara IP
- `aruco_detector` - Detección con posiciones RELATIVAS al robot
- `aruco_navigator` - Navegación basada en visión relativa

**Topics publicados:**
- `/roborescue/robot_pose` (Pose2D) - Siempre (0, 0, 0) - referencia del robot
- `/roborescue/blue_box_pose` (Pose2D) - Posición RELATIVA a robot
- `/roborescue/yellow_box_pose` (Pose2D) - Posición RELATIVA a robot

---

### Raspberry Pi 4 (Ubuntu 22.04 + ROS2 Humble)

#### Paquete: `rpi_relay`

**Nodos:**
- `cmd_vel_relay` - Relay de comandos (reenvía `/cmd_vel_laptop` → `/cmd_vel`)

**Topics:**
- Suscribe: `/roborescue/cmd_vel_laptop` (Twist)
- Publica: `/roborescue/cmd_vel` (Twist) - Para ESP32 vía micro-ROS

#### Paquete: `micro_ros_agent`

**Función:** Puente de comunicación ROS2 ↔ micro-ROS

**Comando:**
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

---

### ESP32 (PlatformIO + micro-ROS)

#### Proyecto: `microros_esp_eurobot`

**Archivo principal:** `src/main.cpp`

**Funcionalidades:**
- Cliente micro-ROS
- Suscripción a `/cmd_vel`
- Cinemática inversa Mecanum
- Control PWM de 4 motores
- Watchdog de seguridad (timeout 500ms)

**Configuración Hardware:**
- **Transporte:** Serial (115200 baud) o WiFi
- **Pines PWM:** GPIO 25, 27, 18, 32
- **Pines DIR:** GPIO 26, 14, 19, 33
- **Canales PWM:** 4 (1 por motor)

**Dependencias (platformio.ini):**
- `micro_ros_platformio`
- Arduino framework

---

## Protocolos de Comunicación

### ROS2 DDS (Portátil ↔ Raspberry Pi)

**Transporte:** WiFi  
**Middleware:** DDS (Fast-RTPS por defecto)  
**Discovery:** Automático en la misma red

**Configuración de red:**
```bash
# Asegurar que ambos dispositivos están en la misma red WiFi
# Opcional: Configurar ROS_DOMAIN_ID para aislar
export ROS_DOMAIN_ID=42
```

**QoS Profiles:**
- Sensores (cámara): `BEST_EFFORT`, `VOLATILE`
- Control (cmd_vel): `RELIABLE`, `VOLATILE`

---

### micro-ROS (Raspberry Pi ↔ ESP32)

**Opción 1: Serial (Recomendado)**

```
RPI4 USB ───(Serial 115200)───► ESP32 UART0
```

**Ventajas:**
- Conexión estable
- Latencia baja y predecible
- No depende de WiFi

**Configuración RPI4:**
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

**Configuración ESP32:**
```cpp
Serial.begin(115200);
set_microros_serial_transports(Serial);
```

---

**Opción 2: WiFi (Alternativa)**

```
RPI4 WiFi ───(UDP)───► ESP32 WiFi
```

**Ventajas:**
- Sin cables
- Más flexible para debugging

**Desventajas:**
- Mayor latencia
- Puede tener pérdidas de paquetes

---

## Estrategia de Control

### Control de Velocidad (Twist)

**Mensaje:** `geometry_msgs/Twist`

```
linear:
  x: velocidad adelante/atrás (m/s)
  y: velocidad lateral (m/s)
  z: 0 (no usado)
angular:
  x: 0 (no usado)
  y: 0 (no usado)
  z: velocidad angular (rad/s)
```

### Cinemática Mecanum (X-Drive)

**Ecuaciones:**
```
vFL = vx - vy - wz
vFR = vx + vy + wz
vRL = vx + vy - wz
vRR = vx - vy + wz
```

**Normalización:**
Si `max(|vFL|, |vFR|, |vRL|, |vRR|) > 1.0`, dividir todas por el máximo.

### Control de Posición

**Algoritmo:** Control basado en visión (Visual Servoing)

1. Detectar posición del robot (ArUco ID=1)
2. Calcular error en coordenadas globales: `error = target - current`
3. Transformar a coordenadas del robot (rotación por -θ)
4. Aplicar control proporcional: `v = Kp * error`
5. Publicar en `/cmd_vel`

**Parámetros:**
- `KP_LIN = 1.5` (ganancia lineal)
- `DISTANCE_STOP = 0.15m` (distancia de parada)

---

## Seguridad y Robustez

### Watchdog en ESP32

**Timeout:** 500ms sin recibir `/cmd_vel` → Parar todos los motores

```cpp
if (millis() - last_msg_time > 500) {
  // Detener todos los motores
  set_motor(ch_FL, FL_DIR, 0);
  set_motor(ch_FR, FR_DIR, 0);
  set_motor(ch_RL, RL_DIR, 0);
  set_motor(ch_RR, RR_DIR, 0);
}
```

### Zona Muerta de Motores

**Threshold:** PWM < 25 → PWM = 0

Evita enviar señales insuficientes que podrían dañar los motores.

### Límites de Velocidad

**Máxima velocidad normalizada:** 1.0 (100% PWM)  
**En m/s:** Depende de calibración (a medir)

---

## Escalabilidad y Extensiones Futuras

### Sensores Adicionales

- **LIDAR 2D:** Para detección de obstáculos robusta
- **IMU:** Para mejorar odometría
- **Sensores de distancia:** Ultrasonido/IR para colisiones

**Integración:** Nuevos nodos ROS2 que publican en topics estándar

### Actuadores Adicionales

- **Servomotores:** Para manipulación de objetos
- **Sistemas neumáticos:** Si el reglamento lo permite

**Integración:** Extender control en ESP32 o añadir Arduino secundario

### SIMA (Small Independent Mobile Actuator)

**Descripción:** Robot pequeño secundario (Ø100mm)

**Arquitectura:**
- ESP32 independiente con micro-ROS
- Comunicación WiFi con RPI4
- Estrategia coordinada con robot principal

---

## Herramientas de Debug

### Visualización

**RViz2:**
```bash
ros2 run rviz2 rviz2
```

**rqt_image_view:**
```bash
ros2 run rqt_image_view rqt_image_view
```

### Monitoreo de Topics

```bash
# Ver todos los topics
ros2 topic list

# Monitorear cmd_vel
ros2 topic echo /cmd_vel

# Medir frecuencia
ros2 topic hz /cmd_vel
```

### Logs

```bash
# Ver logs de un nodo
ros2 node info /navigation_controller

# Grabar topics para análisis
ros2 bag record -a  # Grabar todos
ros2 bag play <bag_file>  # Reproducir
```

---

## Diagrama de Estados (Partido)

```
    ┌─────────┐
    │  INIT   │
    └────┬────┘
         │
         ▼
    ┌─────────┐
    │  READY  │◄──────────┐
    └────┬────┘           │
         │                │
         ▼ (señal inicio) │
    ┌─────────┐           │
    │ RUNNING │           │
    └────┬────┘           │
         │                │
         ├─► NAVIGATE ───►│
         ├─► COLLECT  ───►│
         ├─► SCORE    ───►│
         └─► AVOID    ───►│
         │                │
         ▼ (timeout/end)  │
    ┌─────────┐           │
    │  STOP   │           │
    └────┬────┘           │
         │                │
         ▼ (emergency)    │
    ┌─────────┐           │
    │  E-STOP │───────────┘
    └─────────┘
```

---

## Referencias Técnicas

- [ROS2 Humble Docs](https://docs.ros.org/en/humble/)
- [micro-ROS Docs](https://micro.ros.org/)
- [OpenCV Python](https://docs.opencv.org/4.x/)
- [ArUco Markers](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)

---

**Versión:** 1.0  
**Fecha:** Febrero 2026  
**Autor:** Equipo Eurobot 2026
