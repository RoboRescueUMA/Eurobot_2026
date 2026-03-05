# Conexión de Encoders - ESP32 RoboRescue

## Especificaciones de los Encoders

- **PPR motor:** 11 pulsos por revolución
- **Relación de reducción:** 1:34
- **PPR eje rueda:** 374 pulsos/vuelta (11 × 34)
- **Resolución con cuadratura (x4):** 1496 counts/vuelta
- **Tipo:** Encoders incrementales de doble canal (A y B)

## Pines de cada Motor

Cada motor tiene 6 cables:
- Motor + (conectado al puente H)
- Motor - (conectado al puente H)
- Encoder GND
- Encoder +  (3.3V o 5V)
- Encoder A (fase A)
- Encoder B (fase B)

## Conexiones ESP32

### Alimentación de Encoders
Todos los encoders comparten:
- **Encoder GND** → GND del ESP32
- **Encoder +** → 3.3V del ESP32 (⚠️ Verificar si los encoders soportan 3.3V, si no usar 5V)

### Canales A y B por Motor

⚠️ **IMPORTANTE:** Se evitan GPIO 12 y 15 porque son sensibles durante el boot del ESP32 y causan problemas de programación cuando el robot está conectado.

| Motor | Posición | Canal A | Canal B |
|-------|----------|---------|---------|
| FL    | Frontal Izquierda | GPIO 34 | GPIO 35 |
| FR    | Frontal Derecha   | GPIO 21 | GPIO 22 |
| RL    | Trasera Izquierda | GPIO 23 | GPIO 4  |
| RR    | Trasera Derecha   | GPIO 16 | GPIO 17 |

### Esquema de Conexión

```
MOTOR FL (Frontal Izquierda):
  Motor + ────→ Puente H Driver 2 (ya conectado)
  Motor - ────→ Puente H Driver 2 (ya conectado)
  Encoder GND ─→ GND ESP32
  Encoder +  ──→ 3.3V ESP32
  Encoder A ───→ GPIO 34
  Encoder B ───→ GPIO 35

MOTOR FR (Frontal Derecha):
  Motor + ────→ Puente H Driver 2 (ya conectado)
  Motor - ────→ Puente H Driver 2 (ya conectado)
  Encoder GND ─→ GND ESP32
  Encoder +  ──→ 3.3V ESP32
  Encoder A ───→ GPIO 21
  Encoder B ───→ GPIO 22

MOTOR RL (Trasera Izquierda):
  Motor + ────→ Puente H Driver 1 (ya conectado)
  Motor - ────→ Puente H Driver 1 (ya conectado)
  Encoder GND ─→ GND ESP32
  Encoder +  ──→ 3.3V ESP32
  Encoder A ───→ GPIO 23
  Encoder B ───→ GPIO 4

MOTOR RR (Trasera Derecha):
  Motor + ────→ Puente H Driver 1 (ya conectado)
  Motor - ────→ Puente H Driver 1 (ya conectado)
  Encoder GND ─→ GND ESP32
  Encoder +  ──→ 3.3V ESP32
  Encoder A ───→ GPIO 16
  Encoder B ───→ GPIO 17
```

## Configuración en el Código

El archivo `esp32_roborescue/src/main.cpp` incluye:

1. **Definición de pines** (líneas 32-45)
2. **Interrupciones ISR** para lectura en cuadratura
3. **Cálculo de velocidad** cada 50ms
4. **Publicación ROS** en `/roborescue/encoder_velocities`

### Mensaje ROS Publicado

**Topic:** `/roborescue/encoder_velocities`  
**Tipo:** `std_msgs/Float32MultiArray`  
**Contenido:** `[rpm_FL, rpm_FR, rpm_RL, rpm_RR]`

Velocidades de las 4 ruedas en RPM (revoluciones por minuto).

## Guía de Pruebas

### 1. Compilar y Subir el Código

```bash
cd ~/Desktop/GitHub/pruebas_eurobot/esp32_roborescue
pio run --target upload
```

### 2. Verificar Monitor Serial

```bash
pio device monitor
```

**Salida esperada:**
```
========================================
   INICIANDO ESP32 MECANUM
========================================
✅ Hardware configurado
Configurando encoders...
✅ Encoders configurados
...
Encoders RPM -> FL:0.00 FR:0.00 RL:0.00 RR:0.00
```

### 3. Verificar Topic ROS

```bash
export ROS_DOMAIN_ID=17
ros2 topic echo /roborescue/encoder_velocities
```

**Salida esperada (robot detenido):**
```yaml
data:
- 0.0  # FL
- 0.0  # FR
- 0.0  # RL
- 0.0  # RR
```

### 4. Probar Durante Movimiento

**Configurar sistema:**

```bash
# Terminal 1 (RPI): micro-ROS agent
export ROS_DOMAIN_ID=17
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

# Terminal 2 (RPI): Relay
export ROS_DOMAIN_ID=17
ros2 run rpi_relay cmd_vel_relay

# Terminal 3: Monitorear encoders
export ROS_DOMAIN_ID=17
ros2 topic echo /roborescue/encoder_velocities
```

**Enviar comandos de prueba:**

```bash
export ROS_DOMAIN_ID=17

# Avance frontal
ros2 topic pub --once /roborescue/cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Movimiento lateral
ros2 topic pub --once /roborescue/cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.0, y: 0.2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Rotación
ros2 topic pub --once /roborescue/cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# Detener
ros2 topic pub --once /roborescue/cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Valores Esperados

**Avance frontal (linear.x = 0.2):**
- Las 4 ruedas con valores similares (~30-60 RPM)

**Lateral (linear.y = 0.2):**
- FL y RR: mismo signo
- FR y RL: signo opuesto

**Rotación (angular.z = 0.5):**
- FL y RL: mismo signo
- FR y RR: signo opuesto

### Interpretación de RPM

**Cálculo:**
```
RPM = (counts_en_50ms × 1000 / 50) × 60 / 1496
```

**Rangos típicos:**
- Robot detenido: 0.0 RPM
- Velocidad baja (0.1 m/s): 15-30 RPM
- Velocidad media (0.2 m/s): 30-60 RPM
- Velocidad alta (0.3 m/s): 60-90 RPM

## Notas Técnicas

### Voltaje de Encoders
- **3.3V:** Opción más segura, menor consumo
- **5V:** Usar si encoders requieren 5V (verificar datasheet)

### GPIO Sensibles Durante Boot
- **GPIO 12 y 15:** Evitados por causar problemas de flash
- **Solución:** Usar GPIO 21, 22, 23 en su lugar

### Dirección de Rotación
Si un encoder cuenta en dirección inversa:
- Intercambiar físicamente cables A y B de ese encoder

### Interferencias
Para reducir ruido electromagnético:
- Añadir condensadores 100nF entre A/B y GND
- Separar cables de encoders de cables de potencia
- Usar cable apantallado si es posible

---

**Troubleshooting:** Ver `docs/TROUBLESHOOTING.md` para solución de problemas comunes.

**Última actualización:** Febrero 2026
