# Guía de Instalación de Dependencias

## 📦 Dependencias Externas del Proyecto

Este proyecto requiere paquetes externos que **NO** están incluidos en el repositorio (son clones de Git). Esta guía te ayudará a instalarlos correctamente.

---

## 🖥️ Instalación en Laptop (Ubuntu/Linux)

### Prerequisitos

```bash
# ROS2 Humble debe estar instalado
source /opt/ros/humble/setup.bash

# Dependencias básicas
sudo apt update
sudo apt install -y python3-pip python3-opencv python3-colcon-common-extensions
sudo apt install -y ros-humble-cv-bridge ros-humble-image-transport
```

### Crear Workspace

```bash
cd ~/Desktop/GitHub/pruebas_eurobot
# El workspace ya existe, solo necesitas instalar dependencias externas si las necesitas
```

---

## 🤖 Instalación en Raspberry Pi 4

### 1. ROS2 Humble (si no está instalado)

```bash
# Añadir repositorio de ROS2
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release

# Añadir clave GPG
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Añadir repositorio
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Instalar ROS2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop  # O ros-humble-ros-base para versión ligera
```

### 2. Dependencias Básicas

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Herramientas de desarrollo
sudo apt install -y python3-pip python3-colcon-common-extensions
sudo apt install -y python3-rosdep python3-vcstool

# Librerías de visión y cámara
sudo apt install -y python3-opencv
sudo apt install -y ros-humble-cv-bridge ros-humble-image-transport
sudo apt install -y libcamera-dev libcamera-apps
```

### 3. micro-ROS Agent (Obligatorio para ESP32)

El **micro-ROS agent** es el puente entre ROS2 (RPI) y micro-ROS (ESP32).

```bash
# Crear workspace para micro-ROS
mkdir -p ~/microros_ws/src
cd ~/microros_ws/src

# Clonar micro-ROS setup
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git

# Volver al workspace
cd ~/microros_ws

# Instalar dependencias
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -y

# Compilar
colcon build
source install/setup.bash

# Crear el agent
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh

# Añadir a .bashrc para autoload
echo "source ~/microros_ws/install/setup.bash" >> ~/.bashrc
```

**Uso del agent:**
```bash
# Ejecutar micro-ROS agent (conectado a ESP32 por USB)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

### 4. camera_ros (Opcional - Solo si usas cámara RPi nativa)

⚠️ **NOTA:** Este paquete es solo para usar la cámara física de Raspberry Pi (ribbon cable). 
Para cámaras IP (como tu móvil), **NO** es necesario.

```bash
# Crear workspace
cd ~/Desktop
mkdir -p rpi_camera_ws/src
cd rpi_camera_ws/src

# Clonar repositorio
git clone https://github.com/christianrauch/camera_ros.git

# Volver al workspace
cd ~/rpi_camera_ws

# Instalar dependencias
rosdep install --from-paths src --ignore-src -y

# Compilar
colcon build --symlink-install

# Source
source install/setup.bash

# Añadir a .bashrc
echo "source ~/rpi_camera_ws/install/setup.bash" >> ~/.bashrc
```

**Uso:**
```bash
# Listar cámaras disponibles
ros2 run camera_ros list_cameras

# Lanzar cámara
ros2 run camera_ros camera_node --ros-args -p width:=640 -p height:=480
```

---

## 📦 Resumen de Paquetes Externos

| Paquete | Necesario | Dónde | Repositorio |
|---------|-----------|-------|-------------|
| **micro_ros_setup** | ✅ Sí (RPI) | Raspberry Pi 4 | https://github.com/micro-ROS/micro_ros_setup |
| **micro_ros_agent** | ✅ Sí (RPI) | Se instala con setup | Parte de micro_ros_setup |
| **camera_ros** | ❌ Opcional | Raspberry Pi 4 | https://github.com/christianrauch/camera_ros |

### ¿Cuándo usar cada paquete?

**micro_ros_setup + agent:**
- Siempre necesario en RPI para comunicarse con ESP32
- Obligatorio para control de motores

**camera_ros:**
- Solo si usas cámara ribbon de Raspberry Pi
- **NO necesario** para cámaras IP (móvil con IPCamera app)
- En este proyecto: NO necesario (usamos cámara IP del móvil)

---

## 🔧 Compilar el Proyecto Principal

Una vez instaladas las dependencias externas:

### En Raspberry Pi:

```bash
cd ~/Desktop/ros2_pi_esp  # O tu ruta del proyecto en RPI
source /opt/ros/humble/setup.bash
source ~/microros_ws/install/setup.bash  # Si instalaste micro-ROS

# Compilar
colcon build --symlink-install

# Source
source install/setup.bash

# Añadir a .bashrc para autoload
echo "source ~/Desktop/ros2_pi_esp/install/setup.bash" >> ~/.bashrc
```

### En Laptop:

```bash
cd ~/Desktop/GitHub/pruebas_eurobot
source /opt/ros/humble/setup.bash

# Compilar
colcon build --symlink-install

# Source
source install/setup.bash
```

---

## 🚀 Verificación de Instalación

### Verificar ROS2:
```bash
ros2 --version
# Debe mostrar: ros2 cli version 0.18.x (Humble)
```

### Verificar micro-ROS agent (en RPI):
```bash
ros2 run micro_ros_agent micro_ros_agent --help
# Debe mostrar ayuda del comando
```

### Verificar OpenCV:
```bash
python3 -c "import cv2; print(cv2.__version__)"
# Debe mostrar versión (ej: 4.5.4)
```

### Verificar cv_bridge:
```bash
python3 -c "from cv_bridge import CvBridge; print('OK')"
# Debe mostrar: OK
```

---

## ⚠️ Troubleshooting

### Error: "micro_ros_agent: command not found"

**Solución:**
```bash
source ~/microros_ws/install/setup.bash
# O reinstalar siguiendo pasos de la sección 3
```

### Error: "No module named 'cv2'"

**Solución:**
```bash
sudo apt install -y python3-opencv
# O con pip:
pip3 install opencv-python
```

### Error: "/dev/ttyUSB0: Permission denied"

**Solución:**
```bash
sudo chmod 666 /dev/ttyUSB0
# O permanente:
sudo usermod -a -G dialout $USER
# (requiere logout/login)
```

### Error: "libcamera not found" (solo camera_ros)

**Solución:**
```bash
sudo apt install -y libcamera-dev libcamera-apps
```

---

## 📋 Checklist de Instalación

### Raspberry Pi 4:
- [ ] ROS2 Humble instalado
- [ ] micro_ros_setup clonado y compilado
- [ ] micro-ROS agent funciona
- [ ] OpenCV y cv_bridge instalados
- [ ] Workspace del proyecto compilado
- [ ] .bashrc actualizado con source

### Laptop:
- [ ] ROS2 Humble instalado
- [ ] OpenCV y cv_bridge instalados
- [ ] Workspace del proyecto compilado

### ESP32:
- [ ] PlatformIO instalado
- [ ] Firmware compilado y flasheado
- [ ] Conectado por USB a RPI

---

## 🔗 Enlaces Útiles

- **ROS2 Humble:** https://docs.ros.org/en/humble/Installation.html
- **micro-ROS:** https://micro.ros.org/
- **micro-ROS ESP32:** https://github.com/micro-ROS/micro_ros_platformio
- **camera_ros:** https://github.com/christianrauch/camera_ros
- **OpenCV ArUco:** https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html

---

**Última actualización:** 2026-02-07  
**Equipo:** RoboRescue
