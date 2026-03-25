🤖 Nodo de Visión Eurobot 2026 (ROS 2)

Este paquete de ROS 2 se encarga de la localización del robot en la arena de Eurobot utilizando la cámara y los marcadores ArUco. Calcula la posición exacta ($X, Y$) y la orientación ($\theta$) del robot en tiempo real.

📋 Requisitos Previos

El sistema debe tener instalado:

ROS 2 (Humble o compatible).

OpenCV con soporte para ArUco (python3-opencv o opencv-contrib-python).

Librería NumPy.

⚙️ Configuración (IMPORTANTE)

Para que el nodo funcione en el portátil del robot, se deben revisar estas líneas en vision_node.py:

1. Modo Real (Línea 74)

Cambiar a False para activar la cámara USB. (El True hace que lea una foto en vez de la imagen de la cámara).

self.modo_simulacion = False


2. Rutas (Línea 77)

Ajustar la ruta absoluta a la carpeta donde están las matrices de calibración (.npy).

self.base_dir = "/home/USUARIO/eurobot_ws/src/eurobot_vision/eurobot_vision"


3. ID del Robot (Línea 112)

Confirmar que el ID coincide con el marcador físico del robot.

self.ROBOT_ID = 4


🚀 Ejecución Paso a Paso

1. Compilar el paquete

cd ~/eurobot_ws
colcon build
source install/setup.bash


2. Sincronizar Red (DOMAIN_ID)

Asegúrate de que el portátil y la Raspberry Pi usen el mismo ID:

export ROS_DOMAIN_ID=5


3. Lanzar el nodo

ros2 run eurobot_vision vision_node


📡 Comunicación (Topics)

El nodo publica la pose en el siguiente topic:

Topic: /robot_pose

Tipo de mensaje: geometry_msgs/msg/Pose2D

Frecuencia: 30 Hz

Datos: x (cm), y (cm), theta (radianes).

Nota para C++: El suscriptor en la Raspberry Pi debe incluir el header #include <geometry_msgs/msg/pose2d.hpp>.
