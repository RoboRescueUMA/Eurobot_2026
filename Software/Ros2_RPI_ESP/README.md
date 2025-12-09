# EuroBot_2020. Versión ROS 2 + micro-ros.

## Explicación del prototipo:
1. **Raspberry Pi 4:** procesa imágenes, detecta el código aruco y calcula la velocidad.
2. **ESP32:** recibe los comandos de velocidad ('cmd_vel') y controla los motores DC.

## Requisitos de Hardware

* **Raspberry Pi 4** (Ubuntu Server 22.04 + ROS 2 Humble).
* **ESP32** (con firmware Micro-ROS cargado).
* **Cámara CSI** (Raspberry Pi Camera Module).
* Driver de Motores L298N.
* Chasis con 2 motores DC.

## Guía de instalación desde cero 
1. Preparar las herramientas del sistema:

```bash
sudo apt update
sudo apt install -y python3-opencv python3-serial python3-pip ros-humble-image-transport-plugins git
pip install esptool
```
2. Crear el espaio de trabajo:

```bash
mkdir -p ~/ros2_ws
cd ~/ros2_ws
```
3. Descargar el agente micro-ros (dentro de ~/ros2_ws/src) y el driver de la cámara:

```bash
git clone -b humble [https://github.com/micro-ROS/micro_ros_setup.git](https://github.com/micro-ROS/micro_ros_setup.git)
git clone [https://github.com/christianrauch/camera_ros.git](https://github.com/christianrauch/camera_ros.git)
```
4. Descargar este repositorio (en otra carpeta fuera del ws):


```bash
cd ~
git clone https://github.com/RoboRescueUMA/Eurobot_2026.git
```
5. Instalación de firmware esp32:
	1. Instalar VS Code.
	2. En la pestaña de extensiones, busca **PlatformIO IDE**, instala y luego reinicia VS Code.
	3. Dale al icono de PlatformIO IDE, en quick acces (la barrita de arriba) dale a PIO HOME.
	4. Open Project:
		Selecciona la carpeta Eurobot_2026/Software/Ros2_RPI_ESP/microros_esp
	5.  Conecta la **ESP32** al ordenador con un cable USB.
	6.  Espera un momento a que PlatformIO termine de cargar (verás una barra de progreso abajo o un relojito).
	7.  Mira la **barra azul** en la parte inferior de la ventana.
	8.  Busca y haz clic en el icono de la **Flecha hacia la derecha (→)** (Si pasas el ratón por encima dice `PlatformIO: Upload`).

6. Copiar los paquetes en el entorno de trabajo:

```bash 
cp  ~/Eurobot_2026/Software/Ros2_RPI_ESP/src/* ~/ros2_ws
```

7. (Opcional). Eliminar el repositorio:

```bash
rm -rf ~/Eurobot_2026
```

8. Compilación:

```bash
cd ~/ros2_ws

# 1. Compilar lo básico
colcon build
source install/setup.bash

# 2. Construir el Agente de Micro-ROS (Solo la primera vez)
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/setup.bash

# 3. Compilar nuestro robot
colcon build --packages-select robot_vision
source install/setup.bash

sudo chmod 666 /dev/ttyUSB0
```

9. Ejecución:

Modo perseguidor:
Cuando la cámara detecta un código aruco avanzará en línea recta hasta que esté cerca, entonces se dentendrá.

```bash
ros2 launch robot_vision robot_perseguidor.launch.py
```

Modo manual:
Activaremos los motores tras reiniciar la esp32 para que lo detecte el agente:
```bash
python3 -m esptool --port /dev/ttyUSB0 run
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

Activaremos la cámara:
```bash
ros2 launch robot_vision camera.launch.py
```

Opción perseguidor:
```bash
ros2 run robot_vision aruco_follower.py
```

Opción teclado:
```bash
ros2 run robot_vision aruco_follower.py
```


Si lo vas a probar avísame antes para comprobar que el guión está bien hecho o indícame los errores que encuentres.
