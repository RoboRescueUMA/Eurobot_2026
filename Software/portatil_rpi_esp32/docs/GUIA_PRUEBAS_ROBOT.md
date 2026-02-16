# Guía de Pruebas del Robot - RoboRescue

## 📋 Resumen de Terminales

### Terminal 1 (SSH a RPI): micro-ROS Agent
```bash
cd ~/Desktop/ros2_pi_esp
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

**Salida esperada:**
```
[INFO] [micro_ros_agent]: Serial port: /dev/ttyUSB0
[INFO] [micro_ros_agent]: Running...
[INFO] [TermiosAgentLinux.cpp:228] info | Root.cpp | create_client | create | client_key: 0x...
```

---

### Terminal 2 (SSH a RPI): Comandos de Prueba y Verificación
```bash
cd ~/Desktop/ros2_pi_esp
source install/setup.bash

# Verificar nodos activos
ros2 node list
# Deberías ver: /roborescue/esp32_mecanum

# Verificar tópicos disponibles
ros2 topic list
# Deberías ver: /roborescue/cmd_vel, /parameter_events, /rosout

# Ver información del tópico
ros2 topic info /roborescue/cmd_vel

# Monitorear mensajes en tiempo real (opcional)
ros2 topic echo /roborescue/cmd_vel
```

---

## 🎮 Comandos Completos de Prueba

> **IMPORTANTE:** Ahora el sistema usa namespace `/roborescue/`. Existen dos formas de enviar comandos:
> 1. **Con namespace completo**: `/roborescue/cmd_vel` (recomendado para pruebas directas)
> 2. **Sin namespace**: `/cmd_vel` (deprecated, solo para compatibilidad)

### Prueba 0: Parar motores (SIEMPRE PRIMERO)

**Con namespace (recomendado):**
```bash
ros2 topic pub --once /roborescue/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**Sin namespace (deprecated):**
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

### Prueba 1: Avanzar hacia adelante (100% velocidad)

**Con namespace (recomendado):**
```bash
ros2 topic pub --once /roborescue/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**Sin namespace (deprecated):**
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```
**Resultado esperado:** Los 4 motores giran hacia adelante. El robot avanza.

**Nota:** L298N necesita voltaje alto para vencer la inercia inicial. Si es muy rápido, prueba con 0.8.

---

### Prueba 2: Retroceder (100% velocidad)

**Con namespace (recomendado):**
```bash
ros2 topic pub --once /roborescue/cmd_vel geometry_msgs/msg/Twist "{linear: {x: -1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**Sin namespace (deprecated):**
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**Resultado esperado:** El robot retrocede.

---

### Prueba 3: Giro horario sobre sí mismo (100% velocidad)

**Con namespace (recomendado):**
```bash
ros2 topic pub --once /roborescue/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
```

**Sin namespace (deprecated):**
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
```

**Resultado esperado:** El robot gira en sentido horario sobre su eje central.

---

### Prueba 4: Giro antihorario sobre sí mismo (100% velocidad)

**Con namespace (recomendado):**
```bash
ros2 topic pub --once /roborescue/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.0}}"
```

**Sin namespace (deprecated):**
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.0}}"
```

**Resultado esperado:** El robot gira en sentido antihorario sobre su eje central.

---

### Prueba 5: Desplazamiento lateral derecha (Mecanum - 100%)

**Con namespace (recomendado):**
```bash
ros2 topic pub --once /roborescue/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**Sin namespace (deprecated):**
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**Resultado esperado:** El robot se desplaza lateralmente hacia la derecha sin cambiar orientación.

---

### Prueba 6: Desplazamiento lateral izquierda (Mecanum - 100%)

**Con namespace (recomendado):**
```bash
ros2 topic pub --once /roborescue/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: -1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**Sin namespace (deprecated):**
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: -1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**Resultado esperado:** El robot se desplaza lateralmente hacia la izquierda sin cambiar orientación.

---

### Prueba 7: Movimiento diagonal (adelante + derecha - 80%)

**Con namespace (recomendado):**
```bash
ros2 topic pub --once /roborescue/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.8, y: 0.8, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**Sin namespace (deprecated):**
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.8, y: 0.8, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**Resultado esperado:** El robot se mueve en diagonal (45°) hacia adelante-derecha.

**Nota:** Velocidad reducida a 80% en movimientos combinados para mantener el control.

---

### Prueba 8: Movimiento combinado (adelante + giro - 80%)

**Con namespace (recomendado):**
```bash
ros2 topic pub --once /roborescue/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.8, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.6}}"
```

**Sin namespace (deprecated):**
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.8, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.6}}"
```

**Resultado esperado:** El robot avanza mientras gira (trayectoria curva).

---

### Prueba 9: Modo continuo (mantener movimiento hasta Ctrl+C)

**Con namespace (recomendado):**
```bash
# Quitar --once para envío continuo
ros2 topic pub /roborescue/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**Sin namespace (deprecated):**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**Nota:** Presiona `Ctrl+C` para detener el envío continuo, luego envía comando de PARAR.

---

### Prueba 10: Velocidad media (si 1.0 es muy rápido)

**Con namespace (recomendado):**
```bash
ros2 topic pub --once /roborescue/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.8, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**Sin namespace (deprecated):**
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.8, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```
**Nota:** Usa 0.8 para velocidad controlada si 1.0 es demasiado rápido.

---

## 📖 Explicación del Mensaje Twist

### Estructura del mensaje `geometry_msgs/msg/Twist`

```yaml
linear:
  x: float64  # Velocidad lineal en eje X (adelante/atrás)
  y: float64  # Velocidad lineal en eje Y (derecha/izquierda) - SOLO Mecanum/Omni
  z: float64  # Velocidad lineal en eje Z (arriba/abajo) - No usado en robots terrestres

angular:
  x: float64  # Velocidad angular en eje X (pitch) - No usado
  y: float64  # Velocidad angular en eje Y (roll) - No usado
  z: float64  # Velocidad angular en eje Z (yaw/giro sobre sí mismo)
```

---

### Rangos de Valores

| Campo | Rango | Descripción |
|-------|-------|-------------|
| `linear.x` | `-1.0` a `1.0` | **Positivo:** Adelante<br>**Negativo:** Atrás<br>**0.0:** Sin movimiento |
| `linear.y` | `-1.0` a `1.0` | **Positivo:** Derecha<br>**Negativo:** Izquierda<br>**0.0:** Sin movimiento lateral |
| `linear.z` | `0.0` | No usado en robots terrestres |
| `angular.x` | `0.0` | No usado |
| `angular.y` | `0.0` | No usado |
| `angular.z` | `-1.0` a `1.0` | **Positivo:** Giro horario<br>**Negativo:** Giro antihorario<br>**0.0:** Sin rotación |

---

### Ejemplos Prácticos

#### Movimiento Simple
```yaml
# Avanzar al 50%
linear: {x: 0.5, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}

# Retroceder al 30%
linear: {x: -0.3, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}

# Derecha al 40%
linear: {x: 0.0, y: 0.4, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}

# Giro horario al 50%
linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.5}
```

#### Movimientos Combinados
```yaml
# Diagonal: adelante + derecha
linear: {x: 0.5, y: 0.5, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}

# Curva: adelante + giro
linear: {x: 0.5, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.3}

# Complejo: diagonal + giro
linear: {x: 0.4, y: 0.3, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.2}
```

---

## 🔧 Fórmulas de Control Mecanum (Referencia)

### Configuración del Robot Casa
Según el código en `esp32_casa/src/main.cpp`:

```cpp
// Fórmulas para ruedas Mecanum (configuración X)
float speed_fl = x - y - z;  // Frontal Izquierda (Motor 2)
float speed_fr = x + y + z;  // Frontal Derecha (Motor 3)
float speed_rl = x + y - z;  // Trasera Izquierda (Motor 0)
float speed_rr = x - y + z;  // Trasera Derecha (Motor 1)

Donde:
  x = msg->linear.x   (adelante/atrás)
  y = msg->linear.y   (derecha/izquierda)
  z = msg->angular.z  (giro)
```

### Tabla de Movimientos

| Movimiento | x | y | z | FL | FR | RL | RR |
|------------|---|---|---|----|----|----|----|
| Adelante   | + | 0 | 0 | +  | +  | +  | +  |
| Atrás      | - | 0 | 0 | -  | -  | -  | -  |
| Derecha    | 0 | + | 0 | -  | +  | +  | -  |
| Izquierda  | 0 | - | 0 | +  | -  | -  | +  |
| Giro Horario    | 0 | 0 | + | -  | +  | -  | +  |
| Giro Antihorario| 0 | 0 | - | +  | -  | +  | -  |

---

## 🚀 Secuencia Recomendada para Primera Prueba

1. **Verificar conexión física:**
   - ESP32 alimentado
   - USB conectado a RPI
   - Batería de L298N conectada
   - Motores conectados

2. **Ejecutar micro-ROS agent** (Terminal 1)

3. **Verificar comunicación** (Terminal 2):
   ```bash
   ros2 node list
   ros2 topic list
   ```

4. **Probar movimientos básicos** (en orden):
   - Prueba 0: PARAR (seguridad)
   - Prueba 1: Avanzar
   - Prueba 0: PARAR
   - Prueba 3: Giro
   - Prueba 0: PARAR
   - Prueba 5: Lateral derecha
   - Prueba 0: PARAR

5. **Ajustar si es necesario:**
   - Si un motor gira al revés, invertir cables o modificar código
   - Si la velocidad es muy alta/baja, ajustar los valores

---

## ⚠️ Notas de Seguridad

- **SIEMPRE** ten el comando de PARAR listo para copiar/pegar
- Realiza las primeras pruebas con el robot **elevado** (sin tocar el suelo)
- Los drivers **L298N necesitan voltajes altos** (1.0 o 0.8) para vencer la inercia inicial debido a su caída de voltaje (~2V)
- Con 0.3 o valores bajos, los motores solo zumban sin moverse (voltaje insuficiente)
- Ten espacio libre de al menos 2x2 metros para pruebas en el suelo
- Mantén el botón de emergencia (si lo hay) accesible
- Ten una mano cerca del cable de alimentación para desconectar rápidamente si es necesario

---

## 📊 Registro de Pruebas

### Robot Casa - L298N

| Fecha | Prueba | Resultado | Observaciones |
|-------|--------|-----------|---------------|
| 2026-02-07 | Freno | ✅ OK | Motores se detienen correctamente |
| 2026-02-07 | Avance frontal (0.3) | ⚠️ Insuficiente | Motores zumban pero no se mueven - voltaje insuficiente |
| 2026-02-07 | Avance frontal (1.0) | ✅ OK | Robot avanza correctamente con velocidad alta |
| | | | |

**Conclusión:** L298N requiere velocidades altas (0.8-1.0) debido a caída de voltaje del driver (~2V).

---

## 🔗 Referencias

- Código ESP32 Casa: `esp32_casa/src/main.cpp`
- Especificaciones Hardware: `docs/hardware/ESPECIFICACIONES_HARDWARE.md`
- Comparativa Robots: `docs/hardware/COMPARATIVA_ROBOTS.md`
- Arquitectura Sistema: `docs/architecture/ARQUITECTURA_SISTEMA.md`

---

## 📝 Notas de Actualización

### Cambio de Namespace (2026-02-12)

El sistema ha sido actualizado para usar el namespace `/roborescue/` en lugar de `/robot1/`:

**Antes (deprecated):**
- Nodo: `/microros_esp32_omni`
- Topic: `/cmd_vel`

**Ahora (actual):**
- Nodo: `/roborescue/esp32_mecanum`
- Topic: `/roborescue/cmd_vel`
- ROS_DOMAIN_ID: 17 (configurado en ESP32, RPI y laptop)

**Ventajas del nuevo sistema:**
- ✅ Namespace consistente para competición multi-robot
- ✅ Domain ID 17 configurado para evitar interferencias
- ✅ Preparado para integración con sistema de visión
- ✅ Soporte para futuros sensores y actuadores

**Compatibilidad:**
Los comandos antiguos sin namespace (`/cmd_vel`) siguen funcionando por compatibilidad, pero se recomienda usar el namespace completo (`/roborescue/cmd_vel`) para todas las pruebas nuevas.

---

**Última actualización:** 2026-02-12  
**Autor:** Team RoboRescue
