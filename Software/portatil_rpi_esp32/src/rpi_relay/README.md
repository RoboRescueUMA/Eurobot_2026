# rpi_relay Package

## 📋 Descripción

El paquete **rpi_relay** implementa un **nodo relay simple** que reenvía mensajes de control (`Twist`) desde la **laptop** hacia el **ESP32** a través de la **Raspberry Pi 4**.

**Función:** Puente de comunicación ROS2 que conecta:
- Entrada: `/roborescue/cmd_vel_laptop` (desde laptop vía WiFi)
- Salida: `/roborescue/cmd_vel` (hacia ESP32 vía micro-ROS)

---

## 🏗️ Estructura del Paquete

```
rpi_relay/
├── rpi_relay/                    # Código fuente
│   ├── __init__.py
│   └── cmd_vel_relay.py          # Nodo principal
├── package.xml                   # Metadata del paquete
├── setup.py                      # Configuración Python
├── setup.cfg
└── README.md                     # Este archivo
```

---

## 🚀 Compilación

```bash
# En la raíz del workspace
cd ~/Desktop/GitHub/pruebas_eurobot

# Compilar
colcon build --packages-select rpi_relay

# Fuente
source install/setup.bash
```

---

## 🎯 Nodo Principal: cmd_vel_relay

**Ubicación:** `rpi_relay/cmd_vel_relay.py`

**Propósito:** Reenviar mensajes Twist desde una entrada a una salida con namespace configurable

### Tópicos

| Tópico | Tipo | Dirección | Descripción |
|--------|------|-----------|---|
| `/{namespace}/cmd_vel_laptop` | `geometry_msgs/Twist` | Suscriptor | Comandos desde laptop |
| `/{namespace}/cmd_vel` | `geometry_msgs/Twist` | Publicador | Comandos hacia ESP32 |

### Parámetros

| Parámetro | Tipo | Valor Por Defecto | Descripción |
|-----------|------|------------------|---|
| `namespace` | string | `roborescue` | Namespace para los tópicos |

---

## ▶️ Ejecución

### Modo Simple

```bash
# En RPI, ejecutar:
ros2 run rpi_relay cmd_vel_relay

# Salida esperada:
# ✅ Nodo relay iniciado. Reenviando /roborescue/cmd_vel_laptop → /roborescue/cmd_vel
```

### Con Namespace Personalizado

```bash
ros2 run rpi_relay cmd_vel_relay \
    --ros-args \
    -p namespace:=robot_secondary
```

---

## 🧪 Verificación

```bash
# Ver que el nodo corre
ros2 node list
# Debería mostrar: /cmd_vel_relay

# Ver topics
ros2 topic list | grep cmd_vel
# Debería mostrar: /roborescue/cmd_vel_laptop
# Debería mostrar: /roborescue/cmd_vel

# Monitorear entrada
ros2 topic echo /roborescue/cmd_vel_laptop

# Monitorear salida
ros2 topic echo /roborescue/cmd_vel

# Enviar comando de prueba
ros2 topic pub --once /roborescue/cmd_vel_laptop geometry_msgs/msg/Twist \
    "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## 📊 Topología ROS

```
Laptop (Controller)
    ↓ /roborescue/cmd_vel_laptop
RPI (rpi_relay)
    ↓ /roborescue/cmd_vel
ESP32 (micro-ROS)
    ↓
Motores
```

---

## 🔧 Cómo Funciona

El nodo:
1. **Se suscribe** a `/{namespace}/cmd_vel_laptop`
2. **Recibe** cada mensaje Twist
3. **Lo reenvía sin cambios** a `/{namespace}/cmd_vel`
4. **Logs debug** muestran los valores reenviados (opcional)

**Sin procesamiento adicional** - es un simple pass-through para permitir que la laptop y el ESP32 se comuniquen a través de la RPI.

---

## 🔗 Referencias

- **Guía Completa:** `docs/guias/GUIA_RPI_RELAY.md`
- **Código:** `src/rpi_relay/rpi_relay/cmd_vel_relay.py`
- **Arquitectura:** `docs/architecture/ARQUITECTURA_SISTEMA.md`

---

## 📋 Requisitos

- ROS2 Humble
- Python 3.10+
- Paquete `geometry_msgs`

---

## 📝 Notas

- Este paquete es **muy simple y confiable** - solo reenvía
- **Estateless** - sin memoria de estado
- **Bajo overhead** - mínima latencia
- **Debugging fácil** - monitorea entrada/salida con `ros2 topic echo`

---

**Última actualización:** 2026-03-05  
**Equipo:** RoboRescue
