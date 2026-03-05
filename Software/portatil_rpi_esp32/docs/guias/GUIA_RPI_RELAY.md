# Guía de Uso - RPI Relay (cmd_vel_relay)

## 📋 Descripción General

**RPI Relay** es un nodo ROS2 que actúa como **puente de comunicación** entre los comandos de movimiento enviados desde la **laptop** y el **ESP32** (microcontrolador del robot) conectado a la **Raspberry Pi**.

**Ubicación del código:** `src/rpi_relay/`

---

## 🎯 Función Principal

El nodo reenvía (relay) mensajes `Twist` desde un tópico origen a un tópico de destino:

```
Laptop
    ↓ (publica a /roborescue/cmd_vel_laptop)
RPI Relay (suscriptor)
    ↓ (reenvía)
RPI Relay (publicador)
    ↓ (publica a /roborescue/cmd_vel)
ESP32 (micro-ROS)
    ↓
Motores
```

---

## 🏗️ Estructura

```
rpi_relay/
├── rpi_relay/
│   ├── __init__.py
│   └── cmd_vel_relay.py      # Nodo principal
├── launch/
│   └── rpi_relay.launch.py   # Launch file (si existe)
├── package.xml               # Metadata
├── setup.py                  # Configuración
└── README.md                 # Este archivo
```

---

## 🚀 Instalación y Compilación

### 1. Compilar el Paquete

```bash
# En la raíz del workspace
cd ~/Desktop/GitHub/pruebas_eurobot

# Compilar
colcon build --packages-select rpi_relay

# Fuente
source install/setup.bash
```

### 2. Verificar Compilación

```bash
# Verificar que el paquete es accesible
ros2 pkg list | grep rpi_relay

# Ver el nodo disponible
ros2 run rpi_relay --help
```

---

## 🎮 Uso Básico

### Opción 1: Sin Parámetros (Recomendado)

```bash
# En la RPI, ejecutar:
ros2 run rpi_relay cmd_vel_relay

# Salida esperada:
# ✅ Nodo relay iniciado. Reenviando /roborescue/cmd_vel_laptop → /roborescue/cmd_vel
```

### Opción 2: Con Parámetro de Namespace Personalizado

```bash
# Si quieres usar un namespace diferente (ej: /robot2/)
ros2 run rpi_relay cmd_vel_relay \
    --ros-args \
    -p namespace:=robot2

# Resultado: reenviará de /robot2/cmd_vel_laptop → /robot2/cmd_vel
```

### Opción 3: Con Launch File (Si Existe)

```bash
ros2 launch rpi_relay rpi_relay.launch.py
```

---

## 📊 Topología ROS

### Tópicos

| Tópico | Tipo | Dirección | Descripción |
|--------|------|-----------|---|
| `/roborescue/cmd_vel_laptop` | `geometry_msgs/Twist` | Suscriptor | Comandos desde laptop |
| `/roborescue/cmd_vel` | `geometry_msgs/Twist` | Publicador | Comandos hacia ESP32 |

### Parámetros

| Parámetro | Tipo | Valor Por Defecto | Descripción |
|-----------|------|------------------|---|
| `namespace` | string | `roborescue` | Namespace para los tópicos |

---

## 🔧 Configuración

### Cambiar Namespace

Para usar un robot diferente o un namespace personalizado:

```bash
ros2 run rpi_relay cmd_vel_relay -p namespace:=roborescue_custom
```

Esto reenviará:
- De: `/roborescue_custom/cmd_vel_laptop`
- Hacia: `/roborescue_custom/cmd_vel`

---

## 🧪 Verificación de Funcionamiento

### Test 1: Verificar que el Nodo Corre

```bash
# En otra terminal:
ros2 node list

# Debería mostrar: /cmd_vel_relay
```

### Test 2: Monitorear Topics

```bash
# Terminal 1: Ver datos que llegan al relay
ros2 topic echo /roborescue/cmd_vel_laptop

# Terminal 2: Ver datos que salen del relay
ros2 topic echo /roborescue/cmd_vel

# Enviar un comando de prueba desde laptop:
ros2 topic pub --once /roborescue/cmd_vel_laptop geometry_msgs/msg/Twist \
    "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Resultado esperado: El mismo mensaje debería aparecer en ambas terminales
```

### Test 3: Verificar Información del Topic

```bash
# Ver quién publica y suscribe
ros2 topic info /roborescue/cmd_vel_laptop
ros2 topic info /roborescue/cmd_vel

# Resultado esperado:
# /roborescue/cmd_vel_laptop:
#   Type: geometry_msgs/msg/Twist
#   Publishers: 1      (laptop)
#   Subscribers: 1     (relay)
#
# /roborescue/cmd_vel:
#   Type: geometry_msgs/msg/Twist
#   Publishers: 1      (relay)
#   Subscribers: 1     (ESP32 micro-ROS)
```

---

## 📐 Diagrama de Flujo

```
┌─────────────────────────────────────────────────┐
│            LAPTOP (Controlador)                 │
│                                                 │
│  - Xbox Teleop u otro nodo de control          │
│  - Publica a: /roborescue/cmd_vel_laptop       │
└──────────────┬──────────────────────────────────┘
               │
               │ ROS2 Network (WiFi/Ethernet)
               │ DDS - Domain ID 17
               ↓
┌──────────────────────────────────────────────────┐
│         RASPBERRY PI 4 (Relay)                   │
│                                                  │
│  ┌────────────────────────────────────────────┐ │
│  │   RPI Relay Node                           │ │
│  │                                            │ │
│  │   Suscriptor:                              │ │
│  │   └─ /roborescue/cmd_vel_laptop           │ │
│  │                                            │ │
│  │   Publicador:                              │ │
│  │   └─ /roborescue/cmd_vel                  │ │
│  └────────────────────────────────────────────┘ │
│                                                  │
│   (Ejecuta: ros2 run rpi_relay cmd_vel_relay)  │
└──────────────┬───────────────────────────────────┘
               │
               │ Serial o WiFi micro-ROS
               │
               ↓
┌──────────────────────────────────────────────────┐
│           ESP32 (microcontrolador)               │
│                                                  │
│   - Suscriptor: /roborescue/cmd_vel             │
│   - Cinemática inversa Mecanum                  │
│   - Control PWM de motores                      │
└──────────────────────────────────────────────────┘
```

---

## ⚙️ Integración en el Sistema

### Secuencia de Inicio Recomendada

```bash
# Terminal 1 (En RPI): Iniciar micro-ROS Agent
cd ~/Desktop/ros2_pi_esp
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

# Terminal 2 (En RPI): Iniciar RPI Relay
cd ~/Desktop/GitHub/pruebas_eurobot
source install/setup.bash
export ROS_DOMAIN_ID=17
ros2 run rpi_relay cmd_vel_relay

# Terminal 3 (En Laptop): Iniciar sistema de visión y control
cd ~/Desktop/GitHub/pruebas_eurobot
source install/setup.bash
export ROS_DOMAIN_ID=17
ros2 launch robot_localization robot_localization.launch.py &
ros2 run xbox_controler xbox_teleop  # u otro controlador
```

---

## 🔍 Debugging y Troubleshooting

### Problema: El Relay No Recibe Comandos

**Verificación:**
```bash
# ¿Hay datos en /roborescue/cmd_vel_laptop?
ros2 topic echo /roborescue/cmd_vel_laptop

# Si no hay datos, revisar que el controlador (ej: Xbox Teleop) está publicando ahí
ros2 topic info /roborescue/cmd_vel_laptop
# Debería mostrar: Publishers: 1 (el controlador)
```

**Solución:**
- Verificar que el laptop tiene `ROS_DOMAIN_ID=17`
- Verificar que el controlador está corriendo en la laptop
- Ver logs del relay: `ros2 run rpi_relay cmd_vel_relay --ros-args --log-level DEBUG`

---

### Problema: El ESP32 No Recibe Comandos

**Verificación:**
```bash
# ¿Hay datos saliendo del relay?
ros2 topic echo /roborescue/cmd_vel

# Si no hay datos, el relay no está recibiendo
# Ver logs del relay
```

**Solución:**
- Verificar que el micro-ROS Agent está corriendo en RPI
- Verificar que ESP32 está conectado por USB: `ls /dev/ttyUSB*`
- Verificar que el ESP32 está suscrito: `ros2 node list | grep esp`

---

### Problema: Domain ID Mismatch

**Síntoma:** El relay no se conecta a la laptop o el ESP32

**Solución:**
```bash
# En RPI:
export ROS_DOMAIN_ID=17

# En laptop:
export ROS_DOMAIN_ID=17

# Verificar:
echo $ROS_DOMAIN_ID  # Debe ser 17 en ambas máquinas

# Persistente (.bashrc):
echo "export ROS_DOMAIN_ID=17" >> ~/.bashrc
source ~/.bashrc
```

---

## 📋 Comandos Útiles

```bash
# Ver nodos activos
ros2 node list

# Ver topics disponibles
ros2 topic list

# Ver información de un topic
ros2 topic info /roborescue/cmd_vel_laptop
ros2 topic info /roborescue/cmd_vel

# Monitorear datos en tiempo real
ros2 topic echo /roborescue/cmd_vel_laptop
ros2 topic echo /roborescue/cmd_vel

# Enviar comando de prueba
ros2 topic pub --once /roborescue/cmd_vel_laptop geometry_msgs/msg/Twist \
    "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Matar el nodo
ros2 node kill /cmd_vel_relay

# Ver logs detallados
ros2 run rpi_relay cmd_vel_relay --ros-args --log-level DEBUG
```

---

## 📊 Formatos de Mensajes

### Twist (geometry_msgs/Twist)

Mensaje de control de velocidad:

```yaml
linear:
  x: float64  # Velocidad adelante/atrás (m/s típicamente, pero nosotros usamos normalizado -1.0 a 1.0)
  y: float64  # Velocidad lateral (Mecanum) (normalizado)
  z: float64  # Velocidad vertical (no usado en robots terrestres)
angular:
  x: float64  # Rotación en X (roll) - no usado
  y: float64  # Rotación en Y (pitch) - no usado
  z: float64  # Rotación en Z (yaw/giro) (normalizado -1.0 a 1.0)
```

**Ejemplo de movimiento:**
```bash
# Avanzar rápido
ros2 topic pub /roborescue/cmd_vel_laptop geometry_msgs/msg/Twist \
    "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Girar
ros2 topic pub /roborescue/cmd_vel_laptop geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# Diagonal + giro
ros2 topic pub /roborescue/cmd_vel_laptop geometry_msgs/msg/Twist \
    "{linear: {x: 0.5, y: 0.3, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
```

---

## 🔗 Referencias

- **Código:** `src/rpi_relay/rpi_relay/cmd_vel_relay.py`
- **Arquitectura Sistema:** `docs/architecture/ARQUITECTURA_SISTEMA.md`
- **Guía de Pruebas:** `docs/guias/GUIA_PRUEBAS_ROBOT.md`
- **Xbox Controller:** `docs/guias/GUIA_XBOX_CONTROLLER.md`
- **ROS2 Twist:** https://docs.ros.org/en/humble/p/geometry_msgs/
- **micro-ROS:** https://micro.ros.org/

---

## 📝 Notas

- Este nodo es **muy simple** - solo reenvía mensajes sin modificarlos
- **Sin latencia adicional** - simple pass-through
- **Estateless** - no mantiene estado, cada mensaje es independiente
- **Debugging fácil** - monitorea los topics entrada/salida y verás exactamente qué se pasa

---

**Última actualización:** 2026-03-05  
**Equipo:** RoboRescue
