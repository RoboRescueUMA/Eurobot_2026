# ESP32 - Robot de Pruebas (Casa)

## Descripción
Este es el código para el **robot de pruebas personal** con ruedas preparadas para Mecanum.

---

## Características del Robot

### Hardware
- **Ruedas:** Preparado para 4x Mecanum (código listo aunque aún sin ruedas físicas)
- **Configuración:** X-Drive
- **Drivers:** 2x L298N (puente H clásico)
- **Control:** IN1/IN2/IN3/IN4 (dirección) + ENA/ENB (velocidad PWM)

### Especificaciones Técnicas
- **Movimiento:** Omnidireccional cuando tengas las ruedas Mecanum
- **Control:** Via micro-ROS (igual que RoboRescue)
- **Topic:** `/cmd_vel` (geometry_msgs/Twist)

---

## Configuración de Pines

### Driver 1 (Trasero) - L298N

| Motor | Ubicación | ENA/ENB | IN1 | IN2 | Canal PWM |
|-------|-----------|---------|-----|-----|-----------|
| M0    | Trasero Izquierda | GPIO32 | GPIO33 | GPIO25 | Canal 0 |
| M1    | Trasero Derecha | GPIO14 | GPIO26 | GPIO27 | Canal 1 |

### Driver 2 (Frontal) - L298N

| Motor | Ubicación | ENA/ENB | IN1 | IN2 | Canal PWM |
|-------|-----------|---------|-----|-----|-----------|
| M2    | Frontal Izquierda | GPIO15 | GPIO2 | GPIO4 | Canal 2 |
| M3    | Frontal Derecha | GPIO5 | GPIO18 | GPIO19 | Canal 3 |

**Nota:** Motor 3 usa GPIO18/19 (antes 16/17) para evitar conflictos con UART2.

---

## Lógica de Control

### Driver L298N (IN1/IN2 + EN)

```cpp
// Para cada motor:
IN1=HIGH, IN2=LOW → Adelante
IN1=LOW, IN2=HIGH → Atrás
IN1=LOW, IN2=LOW → Freno
ENA/ENB (PWM 0-255) → Velocidad
```

### Cinemática Mecanum (X-Drive)

```cpp
// Mismo algoritmo que RoboRescue
// Entradas: vx, vy, wz
fl = vx - vy - wz  // Frontal Izquierda (M2)
fr = vx + vy + wz  // Frontal Derecha (M3)
rl = vx + vy - wz  // Trasera Izquierda (M0)
rr = vx - vy + wz  // Trasera Derecha (M1)
```

---

## Diferencias con RoboRescue

| Característica | RoboRescue (Universidad) | Casa (Pruebas) |
|----------------|--------------------------|----------------|
| Driver | DFRobot (PWM+DIR) | L298N (IN1/IN2+EN) |
| Pines por motor | 2 (PWM, DIR) | 3 (IN1, IN2, EN) |
| Lógica control | DIR=HIGH→Adelante | IN1=HIGH,IN2=LOW→Adelante |
| Ruedas físicas | ✅ Mecanum instaladas | ⚠️ Pendiente de instalar |

---

## Compilar y Flashear

```bash
cd ~/Desktop/laptop_rpi_esp/esp32_casa
pio run --target upload
```

### Monitor Serial

```bash
pio device monitor
```

---

## Conectar con ROS2

### Desde tu portátil o RPI:

```bash
# Iniciar micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

### Probar movimiento:

```bash
# Mover adelante
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Mover lateral (cuando tengas las Mecanum)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.3, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Girar
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

---

## Seguridad

### Zona Muerta
- PWM < 20 → PWM = 0 (evita zumbidos y sobrecalentamiento)

### Nodo micro-ROS
- Nombre del nodo: `microros_esp32_omni`
- Suscripción: `/cmd_vel`

---

## Notas Importantes

- ⚠️ **El código está preparado para Mecanum** pero funcionará con ruedas normales (solo se ignorará el movimiento lateral)
- Cuando instales las ruedas Mecanum, el código ya está listo para usarlas sin cambios
- Si tienes problemas con los pines, verifica el pinout específico de tu ESP32
- **Motor 3 cambió de GPIO16/17 a GPIO18/19** para evitar problemas con UART2 (pins RX2/TX2)
- GPIO18/19 son más confiables para control de motores sin restricciones

---

## Historial de Cambios

### 2026-02-07
- **Motor 3 (Frontal Derecha):** Cambiado de GPIO16/17 a GPIO18/19
  - **Razón:** GPIO16 causaba fallo en retroceso (posible conflicto con UART2)
  - **Solución:** GPIOs 18/19 son de propósito general sin restricciones
  - **Acción requerida:** Reconectar cables IN3→GPIO18, IN4→GPIO19

---

## TODO (Cuando lleguen las Mecanum)

- [ ] Instalar las 4 ruedas Mecanum
- [ ] Verificar orientación correcta de las ruedas
- [ ] Calibrar direcciones de motores (puede que necesites invertir alguno)
- [ ] Probar movimiento omnidireccional
- [ ] Ajustar parámetros de normalización si es necesario

---

**Uso:** Pruebas personales y desarrollo  
**Última actualización:** Febrero 2026
