# <span style="font-size:32px;">🤖 IDEA DE MAQUINA DE ESTADOS</span>

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue) ![Python](https://img.shields.io/badge/Python-3.10-green)

<p style="font-size:18px;">Este nodo implementa la lógica completa de misión para un robot en la competición Eurobot 2026, utilizando ROS2 y Nav2 para 
  navegación autónoma y control directo para acciones específicas como el empuje de piezas.</p>


---

## <span style="font-size:24px;">🧠 Descripción General</span>
El sistema se basa en una máquina de estados que permite al robot:
- Detectar su posición inicial
- Determinar el lado del campo (AZUL / AMARILLO)
- Navegar hasta grupos de piezas
- Alinearse correctamente
- Empujar piezas hacia la zona de puntuación
- Repetir el proceso con un segundo grupo
- Aparcar antes de que termine el tiempo

---
## <span style="font-size:24px;">📂 Estructura del nodo</span>

    INIT
     └── ESPERAR_POSE
          ├── F1_RODEAR → F1_POSICION → F1_EMPUJAR
          ├── F2_RODEAR → F2_POSICION → F2_EMPUJAR
          └── APARCAR → COMPLETADO

---

## <span style="font-size:24px;">🔌 Suscripciones</span>

| Topic | Tipo | Descripción |
|-------|------|------------|
| `/robot_pose` | `geometry_msgs/Pose2D` | Pose del robot `(x, y, θ) |

---

## <span style="font-size:24px;">📤 Publicaciones</span>

| Topic | Tipo | Descripción |
|-------|------|------------|
| `/cmd_vel` | `Twist` | Velocidad del robot |
| `/robot_estado` | `String` | Estado actual + tiempo restante |

---

## <span style="font-size:24px;">⏱️ Control de tiempo (Watchdog) (m) </span>

El sistema monitoriza continuamente el tiempo restante:

Si queda menos de TIEMPO_RETORNO:
- Cancela navegación
- Activa retorno de emergencia

Si el tiempo llega a 0:
- Finaliza la misión

## <span style="font-size:24px;">🧭 Navegación </span>
Se utilizan dos modos

## <span style="font-size:24px;">Nav2 </span>

Posiciona el robot DETRÁS de las piezas (robot orientado mirando la zona) además de ayudar a Aparcar. Como nunca he usado Nav2 no se si funcionaría.
```bash
ir_a(x, y, theta)
```
- Usa BasicNavigator
- Incluye timeout de seguridad
- Cancela automáticamente si hay emergencia

## <span style="font-size:24px;"> Control directo (empuje) </span>
cmd_vel a velocidad constante durante el tiempo necesario para cubrir la distancia hasta la zona  (d = v × t, sin odometría).
Se utiliza para empujar las piezas y que el Nav2 no haga que el robot se raye, vaya tambaleando o pare. Tiene el peligro de acumular más error de la cuenta.
```bash
    empujar_ciego(distancia)
```
- Movimiento en línea recta
- Sin planificación
- Velocidad constante
- Independiente de Nav2
- 
## <span style="font-size:24px;"> 🗺️ Sistema de coordenadas del mapa </span>

- Origen : esquina inferior derecha  (X_max, 0)
- X      : crece hacia la izquierda   (AZUL   → X grande,  AMARILLO → X pequeña)
- Y      : crece hacia arriba         (zona de entrega en Y alto)

Convención de ángulos (theta_deg, referencia frame 'map'):
- 90°  = mirando hacia Y+ (hacia la zona de entrega / arriba en imagen)
- 270° = mirando hacia Y- (alejándose de la zona)


## <span style="font-size:24px;"> 📷 Funciones experimentales</span>

Esto tiene como objetivo mirar unicamente la posicion por cámara de vez en cuando (A la hora de Aparcar y alinearse para empujar las piezas) para no acumular demasiado error.
La idea es que vacia el buffer de ROS2 para que la proxima posción que recibamos sea la más reciente.
```bash
    obtener_pose_fresca()
```
- Ideado pero no implementado (Puesto como comentarios en el codigo)
- Pensado para compensar el retardo de la cámara
- Se deberia probar y revisar antes, ya que al ser una idea se la he pedido a Claude y no la he revisado xd.

## <span style="font-size:24px;">🛑 Gestión de Fallos</span>
Esto es un poco experimental también y habría que ver si no nos da problemas o es viable

🚨 Fallos críticos
- f1_posicion
- f2_posicion

Acción:
- Activa emergencia
- Va directamente a aparcar

## <span style="font-size:24px;">📋 Cambios en el Nodo de Visión</span>  
Se han añadido cosas para que el nodo sea compatible con Nav2.

1. Integración con el sistema de transformadas (TF2)
  - Añadido TransformBroadcaster: Se incorporó el emisor de transformadas de ROS2
  - Generación de la TF map -> base_link: El nodo ahora no solo publica un mensaje de posición, sino que inserta al robot en el árbol de coordenadas global. Esto es esencial para que Nav2 sepa dónde está el robot en tiempo real.
  - Conversión a Cuaterniones: Se añadió la función euler_to_quaternion para convertir el ángulo de orientación ($\theta$) al formato que requiere el estándar de ROS 2 (geometry_msgs/Quaternion).

2. Cambio de las medidas de cm -> metros

## <span style="font-size:24px;">❓Como usar el nodo</span>
Instalar dependencias básicas
```bash
        sudo apt install python3-colcon-common-extensions
        sudo apt install python3-rosdep
        sudo rosdep init
        rosdep update
```

<span style="font-size:24px;">Instalar Nav2 -></span>
```bash
    sudo apt install ros-humble-navigation2
    sudo apt install ros-humble-nav2-bringup
```
A la hora de compilar - Lanzar Nav2 (SIM o REAL)
```bash
 # 1. Lanzar Nav2
    ros2 launch nav2_bringup navigation_launch.py use_sim_time:=False
    nodo.nav.waitUntilNav2Active()
```
    
❗ Permisos del script
```bash
    #Si falla ejecución:
    chmod +x fsm.py
```
## <span style="font-size:24px;">📌 Notas importantes</span>

- No se si habría que hacer algo más para que se comunique con el nodo de microros
- Es necesario ajustar a las medias reales del mapa (Donde están las piezas y eso)
- En un futuro habrá que o borrar esta carpeta o hacer un README más serio porque me faltan cosas por decir seguro lol

---
Esto es lo que hace el código basicamente xd
---
<img width="247" height="155" alt="image" src="https://github.com/user-attachments/assets/42807fb0-1877-4038-8110-e7e23ca93887" />
