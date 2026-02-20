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

| Motor | Posición | Canal A | Canal B |
|-------|----------|---------|---------|
| FL    | Frontal Izquierda | GPIO 34 | GPIO 35 |
| FR    | Frontal Derecha   | GPIO 12 | GPIO 13 |
| RL    | Trasera Izquierda | GPIO 15 | GPIO 4  |
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
  Encoder A ───→ GPIO 12
  Encoder B ───→ GPIO 13

MOTOR RL (Trasera Izquierda):
  Motor + ────→ Puente H Driver 1 (ya conectado)
  Motor - ────→ Puente H Driver 1 (ya conectado)
  Encoder GND ─→ GND ESP32
  Encoder +  ──→ 3.3V ESP32
  Encoder A ───→ GPIO 15
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

El archivo `esp32_roborescue/src/main.cpp` ya incluye:

1. **Definición de pines** (líneas 30-45)
2. **Interrupciones ISR** para lectura en cuadratura
3. **Cálculo de velocidad** cada 50ms
4. **Publicación ROS** en `/roborescue/encoder_velocities`

### Mensaje ROS Publicado

**Topic:** `/roborescue/encoder_velocities`
**Tipo:** `std_msgs/Float32MultiArray`
**Contenido:** `[rpm_FL, rpm_FR, rpm_RL, rpm_RR]`

Velocidades de las 4 ruedas en RPM (revoluciones por minuto).

## Verificación de Funcionamiento

### 1. Después de conectar los encoders, subir el código:

```bash
cd esp32_roborescue
pio run --target upload
pio device monitor
```

### 2. En el monitor serial verás:

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

### 3. Mover manualmente las ruedas:

Al girar una rueda a mano, deberías ver cambios en el RPM correspondiente:

```
Encoders RPM -> FL:15.2 FR:0.00 RL:0.00 RR:0.00
```

### 4. Verificar en ROS2:

```bash
export ROS_DOMAIN_ID=17
ros2 topic echo /roborescue/encoder_velocities
```

Deberías ver:
```yaml
data:
- 0.0  # FL
- 0.0  # FR
- 0.0  # RL
- 0.0  # RR
```

## Notas Importantes

⚠️ **Voltaje de Encoders:**
- Si los encoders requieren 5V, conectar Encoder+ a pin 5V del ESP32
- Si soportan 3.3V, usar pin 3.3V (más seguro para GPIO del ESP32)

⚠️ **Dirección de Rotación:**
- Si un motor cuenta en dirección inversa (negativo cuando debería ser positivo), invertir físicamente los cables A y B de ese encoder

⚠️ **Interferencias:**
- Usar cables apantallados para los encoders si hay ruido eléctrico
- Mantener cables de encoders alejados de cables de potencia de motores

## Próximos Pasos

Una vez verificado que los encoders funcionan:

1. ✅ **Odometría:** Calcular posición del robot basándose en encoders
2. ✅ **Control PID:** Implementar control de velocidad en lazo cerrado
3. ✅ **Fusión sensorial:** Combinar ArUco + encoders para mejor localización

---

**Última actualización:** Febrero 2026
