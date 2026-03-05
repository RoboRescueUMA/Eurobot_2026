# Guía de Uso - Xbox Controller (Teleoperación)

## 📋 Descripción General

El Xbox Controller es el dispositivo de **control remoto del robot** mediante teleoperación. Convierte los inputs del joystick Xbox 360/Xbox One en comandos de movimiento (`Twist`) que se envían al robot.

**Ubicación del código:** `src/xbox_controler/`

---

## 🎮 Componentes del Xbox Controller

### Nodo: `xbox_teleop` (`xbox_controler/xbox_teleop.py`)

Nodo ROS2 que:
1. Se suscribe a los inputs del joystick (`/joy` — tópico estándar)
2. Mapea botones y ejes del Xbox a acciones de movimiento
3. Publica comandos `Twist` al robot (por defecto en `/roborescue/cmd_vel`)
4. Implementa botón de seguridad (RB) que debe estar presionado para mover el robot

**Parámetros configurables:**

| Parámetro | Tipo | Valor Por Defecto | Descripción |
|-----------|------|------------------|------------|
| `axis_linear_x` | int | 1 | Eje analógico del stick izquierdo (arriba/abajo) |
| `axis_linear_y` | int | 0 | Eje analógico del stick izquierdo (izq/der) para strafe |
| `axis_angular_yaw` | int | 3 | Eje analógico del stick derecho (izq/der) para giro |
| `enable_button` | int | 5 | Botón de seguridad (5 = RB) |
| `scale_linear_x` | float | 1.0 | Escala/velocidad máxima en eje X |
| `scale_linear_y` | float | -1.0 | Escala/velocidad máxima en eje Y |
| `scale_angular_yaw` | float | -1.0 | Escala/velocidad máxima de rotación |
| `cmd_vel_topic` | string | `/roborescue/cmd_vel` | Tópico de destino para comandos |

---

## 🎯 Mapeo del Xbox Controller

### Botones

| Botón | Acción | Código |
|-------|--------|--------|
| **RB (Botón de Seguridad)** | Debe estar presionado para habilitar movimiento | 5 |
| A | Reservado | 0 |
| B | Reservado | 1 |
| X | Reservado | 2 |
| Y | Reservado | 3 |
| LB | Reservado | 4 |
| Back | Reservado | 6 |
| Start | Reservado | 7 |
| Stick Izq. Click | Reservado | 8 |
| Stick Der. Click | Reservado | 9 |

### Ejes Analógicos

| Eje | Entrada | Rango | Acción |
|-----|---------|-------|--------|
| **Stick Izquierdo (Eje 1)** | Arriba/Abajo | -1.0 a 1.0 | Movimiento adelante/atrás |
| **Stick Izquierdo (Eje 0)** | Izq/Der | -1.0 a 1.0 | Movimiento lateral (strafe) |
| **Stick Derecho (Eje 3)** | Izq/Der | -1.0 a 1.0 | Rotación/giro |
| LT (Gatillo izq.) | Presión | 0.0 a 1.0 | Reservado |
| RT (Gatillo der.) | Presión | 0.0 a 1.0 | Reservado |
| D-Pad | Múltiples | -1.0 a 1.0 | Reservado |

---

## 🚀 Instalación y Configuración

### 1. Instalar Dependencias

```bash
# En Ubuntu/Debian
sudo apt update
sudo apt install -y \
    ros-humble-joy \
    joystick \
    jstest-gtk

# Compilar el paquete
cd ~/Desktop/GitHub/pruebas_eurobot
colcon build --packages-select xbox_controler
source install/setup.bash
```

### 2. Verificar Disponibilidad del Joystick

```bash
# Listar dispositivos conectados
ls -la /dev/input/js*

# Probar el joystick (interactivo)
jstest /dev/input/js0

# O usar la herramienta gráfica
jstest-gtk &
```

Si el joystick no aparece:
- Verifica que está conectado por USB
- Probablemente necesites permisos: `sudo chmod 666 /dev/input/js*`

### 3. Lanzar el Nodo

#### Opción A: Con archivo de configuración

```bash
ros2 launch xbox_controler xbox_launch.py
```

#### Opción B: Sin parámetros (valores por defecto)

```bash
ros2 run xbox_controler xbox_teleop
```

#### Opción C: Con parámetros personalizados

```bash
ros2 run xbox_controler xbox_teleop \
    --ros-args \
    -p scale_linear_x:=0.8 \
    -p scale_angular_yaw:=0.5 \
    -p cmd_vel_topic:=/roborescue/cmd_vel_custom
```

---

## 🎮 Procedimiento de Control

### 1. Secuencia de Inicio

```bash
# Terminal 1: Verificar que joy_node está corriendo
# (Normalmente se inicia automáticamente al conectar el joystick)

# Terminal 2: Iniciar el nodo xbox_teleop
ros2 run xbox_controler xbox_teleop

# Terminal 3 (opcional): Monitorear los comandos que se envían
ros2 topic echo /roborescue/cmd_vel
```

### 2. Controlar el Robot

1. **Conecta el Xbox Controller por USB** a la laptop
2. **Mantén presionado el botón RB** (botón de seguridad)
3. **Mueve el stick izquierdo** (arriba/abajo para avanzar/retroceder)
4. **Mueve el stick izquierdo** (izq/der para strafe lateral)
5. **Mueve el stick derecho** (izq/der para girar)
6. **Suelta RB** para detener el robot

### 3. Seguridad

- **SIEMPRE** mantén el RB presionado mientras controlas
- **Suelta RB** inmediatamente si el robot hace algo inesperado
- El robot se detiene automáticamente si se suelta RB

---

## 📊 Topología ROS

```
Xbox Controller (Hardware)
    ↓
joy_node (/joy)
    ↓
xbox_teleop (este nodo)
    ↓
/roborescue/cmd_vel
    ↓
esp32_mecanum (en RPI vía micro-ROS)
    ↓
Motores del Robot
```

### Tópicos

| Tópico | Tipo | Descripción |
|--------|------|------------|
| `/joy` | `sensor_msgs/Joy` | **Suscriptor.** Entrada bruta del joystick |
| `/roborescue/cmd_vel` | `geometry_msgs/Twist` | **Publicador.** Comandos de velocidad convertidos |

---

## ⚙️ Customización Avanzada

### Cambiar Velocidades Máximas

```bash
# Velocidad más lenta (máximo 0.5 en X y Y)
ros2 run xbox_controler xbox_teleop \
    -p scale_linear_x:=0.5 \
    -p scale_linear_y:=0.5 \
    -p scale_angular_yaw:=0.3
```

### Invertir Controles

```bash
# Invertir stick derecho (giro al revés)
ros2 run xbox_controler xbox_teleop \
    -p scale_angular_yaw:=1.0  # Cambiar de -1.0 a 1.0
```

### Cambiar Botón de Seguridad

```bash
# Usar botón Y (botón 3) como seguridad en lugar de RB (botón 5)
ros2 run xbox_controler xbox_teleop \
    -p enable_button:=3
```

### Cambiar Topic de Destino

```bash
# Publicar a topic personalizado (útil para múltiples robots)
ros2 run xbox_controler xbox_teleop \
    -p cmd_vel_topic:=/robot2/cmd_vel
```

---

## 🔍 Verificación y Diagnóstico

### 1. Verificar que joy_node está activo

```bash
ros2 node list
# Deberías ver: /joy_node
```

### 2. Monitorear inputs del joystick

```bash
ros2 topic echo /joy
# Deberías ver los cambios cuando mueves ejes o presionas botones
```

### 3. Monitorear comandos Twist enviados

```bash
ros2 topic echo /roborescue/cmd_vel
# Deberías ver cambios en linear.x, linear.y, angular.z
```

### 4. Verificar información del topic

```bash
ros2 topic info /roborescue/cmd_vel
# Mostrará publicadores y suscriptores
```

---

## ❌ Solución de Problemas

Consulta `docs/troubleshooting/xbox-controller.md` para problemas comunes y soluciones específicas.

### Problemas Comunes Rápidos

**P: El joystick no es detectado**
- Verifica conexión USB: `lsusb | grep -i joy`
- Probablemente necesites permisos: `sudo chmod 666 /dev/input/js0`
- Reinicia la conexión USB

**P: El robot no responde a los controles**
- Verifica que RB está presionado (botón de seguridad)
- Comprueba que `/roborescue/cmd_vel` tiene suscriptor: `ros2 topic info /roborescue/cmd_vel`
- Verifica que el nodo xbox_teleop está corriendo: `ros2 node list | grep xbox`

**P: Los controles están invertidos o raro**
- Ajusta los parámetros `scale_linear_x`, `scale_linear_y`, `scale_angular_yaw` con valores negativos
- Verifica que los ejes correctos están mapeados: comprueba con `jstest /dev/input/js0`

**P: El robot es demasiado/muy poco sensible**
- Reduce/aumenta los valores `scale_*`
- Ejemplo para reducir sensibilidad a la mitad:
  ```bash
  ros2 run xbox_controler xbox_teleop \
      -p scale_linear_x:=0.5 \
      -p scale_angular_yaw:=0.5
  ```

---

## 📝 Archivo de Configuración (Opcional)

**Ubicación:** `src/xbox_controler/launch/xbox_launch.py`

Edita el archivo launch para configurar parámetros por defecto:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xbox_controler',
            executable='xbox_teleop',
            name='xbox_teleop',
            parameters=[{
                'axis_linear_x': 1,
                'axis_linear_y': 0,
                'axis_angular_yaw': 3,
                'enable_button': 5,
                'scale_linear_x': 0.8,      # Ajusta aquí
                'scale_linear_y': -0.8,     # Ajusta aquí
                'scale_angular_yaw': -0.6,  # Ajusta aquí
                'cmd_vel_topic': '/roborescue/cmd_vel',
            }]
        ),
    ])
```

Luego usa: `ros2 launch xbox_controler xbox_launch.py`

---

## 🔗 Referencias

- Código: `src/xbox_controler/xbox_controler/xbox_teleop.py`
- Launch: `src/xbox_controler/launch/xbox_launch.py`
- Troubleshooting: `docs/troubleshooting/xbox-controller.md`
- Guía de Pruebas Robot: `docs/guias/GUIA_PRUEBAS_ROBOT.md`
- Arquitectura Sistema: `docs/architecture/ARQUITECTURA_SISTEMA.md`

---

**Última actualización:** 2026-03-05  
**Equipo:** RoboRescue
