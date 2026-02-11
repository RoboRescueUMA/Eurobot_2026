# Comparativa de Robots - RoboRescue vs Casa

Este documento explica las diferencias entre los dos robots del proyecto.

---

## Vista General

| Característica | Robot RoboRescue (Universidad) | Robot Casa (Pruebas) |
|----------------|-------------------------------|---------------------|
| **Propósito** | Competición Eurobot 2026 | Desarrollo y pruebas personales |
| **Ubicación código** | `esp32_roborescue/` | `esp32_casa/` |
| **Estado** | ✅ Operativo | ⚠️ En desarrollo |
| **Ruedas** | ✅ 4x Mecanum instaladas | ⏳ Pendiente de recibir |
| **Drivers** | 2x DFRobot | 2x L298N |

---

## Diferencias de Hardware

### Drivers de Motores

#### RoboRescue - DFRobot
```
Control simple: PWM + DIR por motor
- PWM (0-255): Controla velocidad
- DIR (HIGH/LOW): Controla dirección
  - HIGH = Adelante
  - LOW = Atrás

Conexiones por motor: 2 pines
Total pines usados: 8 (4 motores × 2)
```

#### Casa - L298N
```
Control clásico: 2×IN + EN por motor
- IN1/IN2: Controla dirección mediante tabla de verdad
  - IN1=HIGH, IN2=LOW = Adelante
  - IN1=LOW, IN2=HIGH = Atrás
  - IN1=LOW, IN2=LOW = Freno
- EN (PWM 0-255): Controla velocidad

Conexiones por motor: 3 pines (2 por driver compartiendo EN)
Total pines usados: 12
```

---

## Tabla de Pines

### RoboRescue (DFRobot)

| Motor | Ubicación | PWM Pin | DIR Pin | Notas |
|-------|-----------|---------|---------|-------|
| FL | Frontal Izquierda | GPIO18 | GPIO19 | Driver 2 |
| FR | Frontal Derecha | GPIO32 | GPIO33 | Driver 2 |
| RL | Trasera Izquierda | GPIO25 | GPIO26 | Driver 1 |
| RR | Trasera Derecha | GPIO27 | GPIO14 | Driver 1 |

**Total: 8 pines GPIO**

### Casa (L298N)

| Motor | Ubicación | EN Pin | IN1 | IN2 | Notas |
|-------|-----------|--------|-----|-----|-------|
| M2 | Frontal Izquierda | GPIO15 | GPIO2 | GPIO4 | Driver 2 |
| M3 | Frontal Derecha | GPIO5 | GPIO16 | GPIO17 | Driver 2 |
| M0 | Trasera Izquierda | GPIO32 | GPIO33 | GPIO25 | Driver 1 |
| M1 | Trasera Derecha | GPIO14 | GPIO26 | GPIO27 | Driver 1 |

**Total: 12 pines GPIO**

**⚠️ Nota:** GPIO16 y GPIO17 son RX2/TX2 - Verificar que no interfieran con Serial2

---

## Código - Función de Control de Motor

### RoboRescue (DFRobot)

```cpp
void set_motor(int channel, int dir_pin, float speed) {
  // Limitar velocidad
  if (speed > 1.0) speed = 1.0;
  if (speed < -1.0) speed = -1.0;

  int pwm = abs(speed) * 255;
  
  // Zona muerta
  if (pwm < 25) pwm = 0;

  // Control dirección (simple)
  if (speed > 0) {
    digitalWrite(dir_pin, HIGH);  // Adelante
  } else {
    digitalWrite(dir_pin, LOW);   // Atrás
  }
  
  // Aplicar velocidad
  ledcWrite(channel, pwm);
}
```

### Casa (L298N)

```cpp
void controlar_motor(int id, float speed) {
  // Convertir a PWM
  int pwm_value = abs(speed) * 255;
  if (pwm_value > 255) pwm_value = 255;
  
  // Zona muerta
  if (pwm_value < 20) {
    pwm_value = 0;
    speed = 0; 
  }

  int pin_in1, pin_in2, canal_pwm;
  // Asignar pines según ID...
  
  // Control dirección (tabla de verdad)
  if (speed > 0) {
    digitalWrite(pin_in1, HIGH);
    digitalWrite(pin_in2, LOW);   // Adelante
  } 
  else if (speed < 0) {
    digitalWrite(pin_in1, LOW);
    digitalWrite(pin_in2, HIGH);  // Atrás
  } 
  else {
    digitalWrite(pin_in1, LOW);
    digitalWrite(pin_in2, LOW);   // Freno
  }
  
  // Aplicar velocidad
  ledcWrite(canal_pwm, pwm_value);
}
```

---

## Cinemática (Igual en ambos)

Ambos robots usan la misma cinemática Mecanum X-Drive:

```cpp
// Entradas del topic /cmd_vel
float vx = msg->linear.x;   // Adelante/atrás
float vy = msg->linear.y;   // Lateral
float wz = msg->angular.z;  // Rotación

// Cálculo de velocidades por rueda
float vFL = vx - vy - wz;   // Frontal Izquierda
float vFR = vx + vy + wz;   // Frontal Derecha
float vRL = vx + vy - wz;   // Trasera Izquierda
float vRR = vx - vy + wz;   // Trasera Derecha

// Normalización si excede 1.0
float max_val = max(abs(vFL), max(abs(vFR), max(abs(vRL), abs(vRR))));
if (max_val > 1.0) {
  vFL /= max_val;
  vFR /= max_val;
  vRL /= max_val;
  vRR /= max_val;
}
```

---

## Comunicación micro-ROS (Igual en ambos)

Ambos usan el mismo protocolo:

```cpp
// Setup
Serial.begin(115200);
set_microros_serial_transports(Serial);

// Crear nodo
rclc_node_init_default(&node, "nombre_nodo", "", &support);

// Suscribirse a /cmd_vel
rclc_subscription_init_default(&subscriber, &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
  "cmd_vel");
```

**Diferencia:** Nombre del nodo
- RoboRescue: `"esp32_mecanum"`
- Casa: `"microros_esp32_omni"`

---

## Cuándo Usar Cada Uno

### Robot RoboRescue 🏆
**Usar para:**
- ✅ Competición oficial Eurobot 2026
- ✅ Pruebas finales antes de competir
- ✅ Calibración con hardware definitivo
- ✅ Entrenamiento del equipo

**NO usar para:**
- ❌ Experimentos arriesgados
- ❌ Desarrollo de features sin probar
- ❌ Pruebas que puedan dañar hardware

### Robot Casa 🏠
**Usar para:**
- ✅ Desarrollo de nuevas funcionalidades
- ✅ Pruebas de algoritmos de navegación
- ✅ Aprendizaje personal
- ✅ Experimentos con código
- ✅ Depuración antes de pasar a RoboRescue

**Ventaja:** Si algo falla, no afecta al robot de competición

---

## Migración de Código entre Robots

### De Casa → RoboRescue

1. ✅ La cinemática es idéntica (copiar directamente)
2. ✅ La lógica de control ROS es igual
3. ⚠️ Cambiar función de control de motor:
   - `controlar_motor()` (L298N) → `set_motor()` (DFRobot)
4. ⚠️ Actualizar asignación de pines
5. ⚠️ Cambiar nombre del nodo si es necesario

### De RoboRescue → Casa

Proceso inverso, cambiar control de motores y pines.

---

## Recomendaciones

### Workflow Sugerido

1. **Desarrollar en Casa:** Probar nuevas ideas en el robot de pruebas
2. **Verificar código:** Asegurar que funciona correctamente
3. **Migrar a RoboRescue:** Adaptar el código probado al robot de competición
4. **Probar en RoboRescue:** Validar con hardware final
5. **Competir:** Usar solo código estable y probado

### Backup y Seguridad

- Mantener siempre un código funcional de respaldo en RoboRescue
- Hacer commits frecuentes cuando el código funciona
- Documentar cambios importantes
- Probar primero en Casa siempre que sea posible

---

## Troubleshooting

### Problema: Motor gira al revés

**RoboRescue (DFRobot):**
```cpp
// Invertir lógica DIR
digitalWrite(dir_pin, speed > 0 ? LOW : HIGH);  // Invertido
```

**Casa (L298N):**
```cpp
// Intercambiar IN1 e IN2 en la asignación de pines
// O invertir en el código:
if (speed > 0) {
  digitalWrite(pin_in1, LOW);   // Invertido
  digitalWrite(pin_in2, HIGH);  // Invertido
}
```

### Problema: Motor no responde

- Verificar conexiones de pines
- Verificar alimentación del driver
- Comprobar que el canal PWM está bien configurado
- Revisar que la zona muerta no esté muy alta

### Problema: Movimiento lateral incorrecto

- Verificar orientación de ruedas Mecanum
- Comprobar que los motores están en las posiciones correctas (FL, FR, RL, RR)
- Revisar las ecuaciones de cinemática

---

**Última actualización:** Febrero 2026  
**Equipo:** RoboRescue
