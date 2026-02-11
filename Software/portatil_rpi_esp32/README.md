# Robot Eurobot 2026 - Equipo RoboRescue

Proyecto de robot autónomo para la competición **Eurobot 2026** - Categoría Senior.

**Equipo:** RoboRescue

---

## Descripción del Proyecto

Este proyecto implementa un robot autónomo con arquitectura distribuida para competir en Eurobot 2026. El sistema está dividido en tres componentes principales:

- **Portátil**: Procesamiento intensivo de visión por computadora
- **Raspberry Pi 4**: Coordinación, navegación y control de alto nivel
- **ESP32**: Control de bajo nivel de motores y actuadores

### Características Principales

- 🤖 Robot con **ruedas Mecanum** para movimiento omnidireccional
- 👁️ **Sistema de visión distribuida** con detección de **ArUco markers**
- 🎯 **Navegación autónoma** con control proporcional
- 🔗 Comunicación distribuida mediante **ROS2 Humble**
- ⚡ Control de motores con **micro-ROS** en ESP32
- 📍 Localización precisa mediante cámara zenital (IP Camera)
- 🔄 **Arquitectura de 3 capas:** Laptop (visión) → RPI4 (relay) → ESP32 (motores)

---

## Estructura del Proyecto

```
laptop_rpi_esp/
├── docs/                         # Documentación del proyecto
│   ├── reglamento/              # Reglamento oficial Eurobot 2026
│   │   ├── EurobotSenior_general.pdf
│   │   ├── Arena y puntuaciones.pdf
│   │   └── RESUMEN_REGLAMENTO.md
│   ├── hardware/                # Especificaciones técnicas del hardware
│   │   └── ESPECIFICACIONES_HARDWARE.md
│   ├── architecture/            # Diagramas de arquitectura
│   │   └── ARQUITECTURA_SISTEMA.md
│   ├── GUIA_VISION_DISTRIBUIDA.md    # Guía rápida del sistema de visión
│   ├── INSTALACION_DEPENDENCIAS.md   # Instalación de dependencias externas
│   ├── GUIA_PRUEBAS_ROBOT.md         # Comandos de prueba
│   └── TROUBLESHOOTING.md            # Problemas y soluciones
├── src/                         # Paquetes ROS2
│   ├── laptop_vision/          # 🆕 Sistema de visión distribuida (Laptop)
│   │   ├── camera_publisher.py      # Captura de cámara IP
│   │   ├── aruco_detector.py        # Detección de ArUco markers
│   │   ├── aruco_navigator.py       # Control proporcional de navegación
│   │   ├── launch/                  # Launch files
│   │   └── config/camera.yaml       # Configuración de cámara
│   ├── rpi_relay/              # 🆕 Relay de comandos (RPI4)
│   │   └── cmd_vel_relay.py         # Reenvía /robot1/cmd_vel → /cmd_vel
│   ├── robot_vision/           # (Antiguo - backup)
│   ├── robot_navigator/        # (Antiguo - backup)
│   └── interfaces/             # Mensajes y servicios personalizados (TODO)
├── esp32_roborescue/           # ESP32 robot competición (DFRobot drivers)
│   ├── src/main.cpp            # Control Mecanum con PWM+DIR
│   └── README.md               # Documentación específica
├── esp32_casa/                  # ESP32 robot pruebas (L298N drivers)
│   ├── src/main.cpp            # Control Mecanum con IN1/IN2+EN
│   └── README.md               # Documentación específica
├── config/                      # Archivos de configuración
├── launch/                      # Launch files centralizados
└── README.md                    # Este archivo
```

---

## Requisitos del Sistema

### Hardware

- **Portátil** con WiFi (Ubuntu 22.04 recomendado)
- **Raspberry Pi 4** (4GB RAM o superior)
- **ESP32** (DevKit o similar)
- **4x Motores DC** con encoders y reductora
- **2x Puentes H** (drivers DFRobot o equivalentes)
- **4x Ruedas Mecanum** (omni-direccionales)
- **Cámara IP** (WiFi/Ethernet, HD)
- **Marcadores ArUco** (DICT_4X4_50)
- Batería LiPo con protección (máx. 48V según reglamento)

### Software

- **ROS2 Humble** (Ubuntu 22.04)
- **Python 3.10+** con OpenCV, NumPy
- **PlatformIO** (para ESP32)
- **micro-ROS** (para comunicación ESP32-ROS2)

---

## Instalación

### 1. Configurar ROS2 Workspace (Portátil y Raspberry Pi)

```bash
# Instalar ROS2 Humble (si no está instalado)
# Ver: https://docs.ros.org/en/humble/Installation.html

# Clonar el repositorio
cd ~/Desktop/GitHub
git clone <URL_DEL_REPOSITORIO> pruebas_eurobot
cd pruebas_eurobot

# Instalar dependencias de Python y OpenCV
sudo apt update
sudo apt install python3-pip python3-opencv ros-humble-cv-bridge

# Instalar dependencias de Python adicionales
pip3 install opencv-python numpy

# Compilar workspace
colcon build --packages-select laptop_vision rpi_relay
source install/setup.bash
```

**Nota:** Ver `docs/INSTALACION_DEPENDENCIAS.md` para detalles completos de todas las dependencias externas.

### 2. Configurar micro-ROS en Raspberry Pi

#### Opción A: Instalar micro-ROS Agent (solo agent)

```bash
# Instalar micro-ROS agent desde paquetes binarios
sudo apt install ros-humble-micro-ros-agent

# Ejecutar agent (ajustar puerto serial)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

#### Opción B: Compilar micro_ros_setup (completo, requerido para desarrollo ESP32)

```bash
# Crear workspace separado para micro-ROS
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clonar micro_ros_setup
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git

# Compilar
cd ~/ros2_ws
colcon build --packages-select micro_ros_setup
source install/setup.bash

# Ejecutar agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

**Importante:** Si vas a compilar firmware para ESP32, necesitas usar la Opción B.

### 3. Flashear ESP32 (desde cualquier máquina con PlatformIO)

#### Robot RoboRescue (Universidad - Competición):
```bash
cd esp32_roborescue/
# Editar platformio.ini si es necesario
pio run --target upload
```

#### Robot Casa (Pruebas personales):
```bash
cd esp32_casa/
# Editar platformio.ini si es necesario
pio run --target upload
```

**Nota:** Ver README.md en cada carpeta ESP32 para detalles específicos de hardware.

---

## Uso

### Sistema de Visión Distribuida (Recomendado)

Este es el sistema actualmente en uso que implementa visión zenital con ArUco markers.

#### En Portátil (Sistema de Visión):

```bash
cd ~/Desktop/GitHub/pruebas_eurobot
source install/setup.bash

# Configurar el IP de la cámara (app IPCamera en móvil)
# Reemplazar 192.168.100.122:5000 con tu IP:puerto

# Opción 1: Navegar hacia caja azul
ros2 launch laptop_vision laptop_vision.launch.py \
  camera_ip:=192.168.100.122:5000 \
  target:=blue_box

# Opción 2: Navegar hacia caja amarilla
ros2 launch laptop_vision laptop_vision.launch.py \
  camera_ip:=192.168.100.122:5000 \
  target:=yellow_box

# Ver imagen con detecciones en tiempo real
# Opción A: Video con anotaciones ArUco
ros2 run rqt_image_view rqt_image_view /robot1/zenital/debug

# Opción B: Video comprimido (mejor rendimiento por WiFi)
ros2 run rqt_image_view rqt_image_view /robot1/zenital/image_raw/compressed
```

#### En Raspberry Pi 4 (Relay + micro-ROS):

```bash
cd ~/Desktop/GitHub/pruebas_eurobot
source install/setup.bash

# Terminal 1: Micro-ROS Agent (comunicación con ESP32)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

# Terminal 2: Relay de comandos (reenvía /robot1/cmd_vel → /cmd_vel)
ros2 run rpi_relay cmd_vel_relay
```

**Ver:** `docs/GUIA_VISION_DISTRIBUIDA.md` para guía detallada y configuración de ArUco markers.

---

### Sistema Anterior (Backup)

Los paquetes `robot_vision` y `robot_navigator` son versiones anteriores del sistema.

#### En Raspberry Pi 4:

```bash
# Terminal 1: Micro-ROS Agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0

# Terminal 2: Navegación y control
ros2 launch robot_navigator eurobot_launch.py
```

#### En Portátil:

```bash
# Terminal 1: Procesamiento de visión
ros2 launch robot_vision camera_launch.py

# Terminal 2: Monitoreo (opcional)
ros2 run rqt_image_view rqt_image_view
```

---

### Verificar Comunicación

```bash
# Ver topics activos
ros2 topic list

# Monitorear velocidades enviadas a ESP32
ros2 topic echo /cmd_vel

# Ver posiciones detectadas
ros2 topic echo /robot1/robot_pos
ros2 topic echo /robot1/blue_box_pos
ros2 topic echo /robot1/yellow_box_pos

# Ver imágenes procesadas
ros2 run rqt_image_view rqt_image_view /robot1/zenital/debug
```

---

## Arquitectura del Sistema

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    ARQUITECTURA DISTRIBUIDA - 3 CAPAS                        │
└─────────────────────────────────────────────────────────────────────────────┘

  ┌─────────────────────┐        ┌─────────────────────┐        ┌──────────────────┐
  │   PORTÁTIL (WiFi)   │        │   RPI4 (Ethernet)   │        │  ESP32 (Serial)  │
  │                     │        │                     │        │                  │
  │  laptop_vision      │        │   rpi_relay         │        │  micro-ROS       │
  │  ├─ camera_pub      │        │   ├─ cmd_vel_relay  │        │  ├─ Cinemática   │
  │  ├─ aruco_detect◄───┼────────┼──►│   (namespace    │        │  │   Mecanum      │
  │  └─ aruco_nav       │  ROS2  │   │    removal)     │        │  └─ Control PWM  │
  │                     │  WiFi  │   └─────────┬───────┼────────┼─────────►        │
  │  Publicadores:      │  DDS   │             │       │ Serial │                  │
  │  /robot1/zenital/*  │        │   /robot1/  │       │ micro- │   Suscriptor:    │
  │  /robot1/*_pos      │        │   cmd_vel   │       │  ROS   │   /cmd_vel       │
  │  /robot1/cmd_vel... │        │      ↓      │       │        │                  │
  └──────────┬──────────┘        │   /cmd_vel  │       │        └────────┬─────────┘
             │                   └─────────────┘       │                 │
             │                                         │                 │
     Cámara IP (móvil)                         micro_ros_agent    4x Motores DC
     ArUco tracking                            (ROS2↔micro-ROS)   + 2x Drivers
     (192.168.x.x:5000)                        bridge process     (Ruedas Mecanum)
```

### Flujo de Datos Completo

1. **Cámara IP** (móvil con IPCamera app) → Stream video vía WiFi
2. **Portátil** recibe stream → `camera_publisher` publica en `/robot1/zenital/image_raw`
3. **Portátil** `aruco_detector` procesa imagen → Detecta ArUco markers (Robot ID=1, Cajas)
4. **Portátil** `aruco_detector` calcula posiciones relativas → Publica en `/robot1/*_pos`
5. **Portátil** `aruco_navigator` recibe posiciones → Calcula velocidades (control proporcional)
6. **Portátil** `aruco_navigator` publica comandos → `/robot1/cmd_vel_laptop` (Twist)
7. **RPI4** `cmd_vel_relay` reenvía → `/robot1/cmd_vel_laptop` → `/cmd_vel` (sin namespace)
8. **ESP32** recibe vía micro-ROS → `/cmd_vel` (Twist)
9. **ESP32** calcula cinemática inversa → Velocidades individuales de 4 ruedas Mecanum
10. **ESP32** envía PWM → 4 motores DC → Robot se mueve omnidireccionalmente

### Topics ROS2 Principales

**Laptop publica:**
- `/robot1/zenital/image_raw` - Video cámara (Image)
- `/robot1/zenital/image_raw/compressed` - Video comprimido (CompressedImage)
- `/robot1/zenital/debug` - Video con anotaciones ArUco (Image)
- `/robot1/robot_pos` - Posición del robot, siempre (0, 0, 0) como referencia (Pose2D)
- `/robot1/blue_box_pos` - Posición relativa de caja azul (Pose2D)
- `/robot1/yellow_box_pos` - Posición relativa de caja amarilla (Pose2D)
- `/robot1/cmd_vel_laptop` - Comandos de velocidad (Twist)

**RPI4 relay:**
- Suscribe: `/robot1/cmd_vel_laptop` (Twist)
- Publica: `/cmd_vel` (Twist) - Para ESP32 vía micro-ROS

**ESP32 suscribe:**
- `/cmd_vel` (Twist) - Velocidades lineales (x, y) y angular (theta)

---

## Documentación Adicional

### Documentación del Sistema

- 👁️ [**Guía del Sistema de Visión Distribuida**](docs/GUIA_VISION_DISTRIBUIDA.md) - Uso del sistema de visión con ArUco
- 📦 [**Instalación de Dependencias**](docs/INSTALACION_DEPENDENCIAS.md) - Guía completa de dependencias externas
- 🧪 [**Guía de Pruebas**](docs/GUIA_PRUEBAS_ROBOT.md) - Comandos de prueba para Robot Casa y RoboRescue
- 🔧 [**Troubleshooting**](docs/TROUBLESHOOTING.md) - Problemas comunes y soluciones

### Documentación del Proyecto

- 📖 [Resumen del Reglamento Eurobot 2026](docs/reglamento/RESUMEN_REGLAMENTO.md)
- 🔩 [Especificaciones de Hardware](docs/hardware/ESPECIFICACIONES_HARDWARE.md)
- 🏗️ [Arquitectura del Sistema](docs/architecture/ARQUITECTURA_SISTEMA.md)

### Documentación del Hardware

- 🤖 [ESP32 RoboRescue](esp32_roborescue/README.md) - Robot de competición (DFRobot drivers)
- 🏠 [ESP32 Casa](esp32_casa/README.md) - Robot de pruebas (L298N drivers)

---

## Calendario y Tareas Pendientes

### Fechas Importantes (Verificar con organizador)

- ✅ Inscripción del equipo
- ⚠️ **Logo del equipo** - Fecha límite: TBD
- ⚠️ **Video de presentación** - Fecha límite: 10 de Febrero
- ⚠️ **Homologación del robot** - Fecha límite: 1 de Marzo
- ⚠️ **Póster técnico y presentación** - Fecha límite: 1 de Marzo
- 🎯 **Robot montado (hardware)** - Fecha límite: 20 de Febrero
- 🎯 **Software básico funcional** - Fecha límite: 15 de Marzo
- 🎯 **Pruebas en escenario** - Fecha límite: 15 de Abril
- 🏁 **Competición Nacional** - Abril 2026

### Próximos Pasos Técnicos

#### ✅ Completados

- [x] **Sistema de visión distribuida funcional** - Laptop (visión) → RPI4 (relay) → ESP32 (motores)
- [x] **Detección de ArUco markers** - DICT_4X4_50 con auto-calibración de escala
- [x] **Navegación autónoma básica** - Control proporcional hacia cajas azul/amarilla (ambas probadas)
- [x] **Timeout de seguridad** - Robot se detiene automáticamente sin detección (1s)
- [x] **Cámara IP funcional** - Streaming desde móvil con app IPCamera
- [x] **Namespace correcto en ROS2** - Topics organizados bajo `/robot1/`
- [x] **Comunicación RPI ↔ Laptop** - ROS2 DDS sobre WiFi/Ethernet
- [x] **Relay de comandos en RPI** - `/robot1/cmd_vel_laptop` → `/cmd_vel`
- [x] **Conexión ESP32 ↔ RPI (micro-ROS)** - Serial /dev/ttyUSB0 funcionando
- [x] **Integración completa 3 capas** - Sistema end-to-end probado exitosamente
- [x] **Control de motores desde visión** - Robot se mueve según detección ArUco

#### 🔄 En Progreso

- [ ] **Pruebas con ArUco en el robot** - Navegación en suelo con marcador sobre robot móvil
- [ ] **Control de velocidad adaptativo** - Reducir velocidad según distancia al objetivo
- [ ] **Calibración de ganancias PID** - Ajustar `linear_p_gain` y `angular_p_gain` en terreno real
- [x] **Implementar strafing lateral** - Usar `cmd.linear.y` para movimiento omnidireccional Mecanum (código listo, pendiente prueba)

#### 📋 Pendientes

- [ ] **Lectura de encoders en ESP32** - Implementar odometría para cerrar el lazo de control
- [ ] **Añadir límites del área de juego** - Detección de bordes del campo (2m x 3m)
- [ ] **Evitación de obstáculos** - Sensores ultrasónicos o detección por visión
- [ ] **Planificación de trayectorias** - Secuencia de movimientos para múltiples objetivos
- [ ] **Estrategia de juego Eurobot 2026** - Maximizar puntuación según reglamento
- [ ] **Pruebas en escenario impreso** - Campo a escala real (arena 2m x 3m)
- [ ] **Calibración de cámaras** - Corrección de distorsión de lente
- [ ] **Sistema de telemetría** - Dashboard para monitoreo en tiempo real

---

## Referencias del Proyecto Antiguo

El código base proviene de proyectos anteriores:

- **ROS2 Workspace:** `/home/maki/Desktop/rpi_casa/src/`
  - `robot_vision/` - Detección ArUco y seguimiento
  - `robot_navigator/` - Control de navegación
  
- **ESP32 micro-ROS:** `/home/maki/Documents/PlatformIO/Projects/microros_esp_eurobot/`
  - Control de motores Mecanum funcional

---

## Contribuir

Este es un proyecto de equipo para Eurobot 2026.

### Workflow de Desarrollo

1. Crear rama para nueva funcionalidad: `git checkout -b feature/nombre`
2. Desarrollar y probar localmente
3. Commit con mensajes descriptivos
4. Push y crear pull request para revisión
5. Integrar tras aprobación del equipo

---

## Recursos Externos

- 🌐 [Sitio oficial Eurobot](https://www.eurobot.org/)
- 📚 [FAQ Eurobot](https://www.eurobot.org/faq/)
- 📧 Comité de árbitros: referee@eurobot.org
- 🤖 [Documentación ROS2 Humble](https://docs.ros.org/en/humble/)
- 🔌 [micro-ROS Documentation](https://micro.ros.org/)

---

## Licencia

Por definir por el equipo.

---

## Equipo

**Nombre del equipo:** RoboRescue  
**Año:** 2026  
**Categoría:** Senior

**Miembros:**
- [Completar]

---

## Robots del Proyecto

### Robot RoboRescue (Universidad - Competición)
- **Ubicación código:** `esp32_roborescue/`
- **Hardware:** 4x Ruedas Mecanum + 2x DFRobot drivers
- **Estado:** Listo para competición
- **Uso:** Eurobot 2026 oficial

### Robot Casa (Pruebas personales)
- **Ubicación código:** `esp32_casa/`
- **Hardware:** Preparado para Mecanum + 2x L298N drivers
- **Estado:** En desarrollo
- **Uso:** Pruebas y desarrollo de software

---

**Última actualización:** Febrero 7, 2026
