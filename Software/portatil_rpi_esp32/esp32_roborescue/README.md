# ESP32 - Robot RoboRescue (Universidad)

## Descripción
Este es el código para el **robot de la competición Eurobot 2026** del equipo **RoboRescue**.

---

## Características del Robot

### Hardware
- **Ruedas:** 4x Mecanum (omni-direccionales)
- **Configuración:** X-Drive
- **Drivers:** 2x Puente H DFRobot
- **Control:** PWM + DIR por motor

### Especificaciones Técnicas
- **Movimiento:** Omnidireccional (X, Y, rotación)
- **Control:** Via micro-ROS desde Raspberry Pi 4
- **Topic:** `/cmd_vel` (geometry_msgs/Twist)

---

## Configuración de Pines

### Driver 1 (Trasero)

| Motor | Ubicación | PWM Pin | DIR Pin | Canal PWM |
|-------|-----------|---------|---------|-----------|
| RL    | Trasero Izquierda | GPIO25 | GPIO26 | Canal 0 |
| RR    | Trasero Derecha | GPIO27 | GPIO14 | Canal 1 |

### Driver 2 (Delantero)

| Motor | Ubicación | PWM Pin | DIR Pin | Canal PWM |
|-------|-----------|---------|---------|-----------|
| FL    | Frontal Izquierda | GPIO18 | GPIO19 | Canal 2 |
| FR    | Frontal Derecha | GPIO32 | GPIO33 | Canal 3 |

---

## Lógica de Control

### Driver DFRobot (PWM + DIR)

```cpp
// Para cada motor:
PWM (0-255) → Velocidad
DIR HIGH → Adelante
DIR LOW → Atrás
```

### Cinemática Mecanum (X-Drive)

```cpp
// Entradas: vx, vy, wz
fl = vx - vy - wz  // Frontal Izquierda
fr = vx + vy + wz  // Frontal Derecha
rl = vx + vy - wz  // Trasera Izquierda
rr = vx - vy + wz  // Trasera Derecha
```

---

## Compilar y Flashear

```bash
cd ~/Desktop/laptop_rpi_esp/esp32_roborescue
pio run --target upload
```

### Monitor Serial

```bash
pio device monitor
```

---

## Conectar con ROS2

### En Raspberry Pi 4:

```bash
# Iniciar micro-ROS agent (ajustar puerto si necesario)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

### Probar movimiento:

```bash
# Mover adelante
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Mover lateral (izquierda)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.3, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Girar
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

---

## Seguridad

### Watchdog
- Timeout: **500ms** sin recibir mensajes → Parar motores
- Zona muerta: PWM < 25 → PWM = 0

---

## Notas

- Este código está optimizado para los drivers DFRobot usados en el robot de competición
- Para el robot de pruebas casero, usar la carpeta `esp32_casa/`
- Asegurar que los cables están bien conectados según el pinout

---

**Equipo:** RoboRescue  
**Competición:** Eurobot 2026  
**Última actualización:** Febrero 2026
