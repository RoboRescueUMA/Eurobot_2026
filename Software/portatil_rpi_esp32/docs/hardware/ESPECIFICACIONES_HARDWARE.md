# Especificaciones Técnicas del Hardware - Robot Eurobot 2026

## Arquitectura General del Sistema

### Descripción
El robot utiliza una arquitectura **distribuida** basada en tres componentes principales que se comunican mediante ROS2:

```
┌─────────────────┐         ┌─────────────────┐         ┌─────────────────┐
│    PORTÁTIL     │◄───────►│   RASPBERRY PI 4│◄───────►│     ESP32       │
│                 │         │                 │         │                 │
│  Procesamiento  │  WiFi   │  Navegación y   │ Serial/ │   Control de    │
│  de Visión      │  ROS2   │  Coordinación   │ WiFi    │   Motores       │
│                 │         │                 │ microROS│                 │
└─────────────────┘         └─────────────────┘         └─────────────────┘
        │                           │                           │
        │                           │                           │
    Cámara IP                   Cámara zenital            4x Motores DC
                                   ArUco                 con Encoders
```

---

## Componentes Hardware

### 1. Portátil (Procesamiento de Visión)

**Función:** Procesamiento intensivo de imágenes de la cámara IP

**Tareas:**
- Captura y procesamiento de video en tiempo real
- Detección de elementos del juego
- Detección de marcadores ArUco
- Publicación de información procesada vía ROS2

**Conexión:**
- WiFi para ROS2 topics
- Comunicación con cámara IP

---

### 2. Raspberry Pi 4 (Cerebro del Robot)

**Función:** Coordinación, navegación y control de alto nivel

**Especificaciones:**
- Modelo: Raspberry Pi 4 (4GB RAM recomendado)
- OS: Ubuntu 22.04 / ROS2 Humble

**Tareas:**
- Ejecutar nodos de navegación y planificación
- Recibir datos procesados del portátil
- Enviar comandos de velocidad a ESP32
- Ejecutar micro-ROS agent
- Localización del robot (usando cámara zenital + ArUco)

**Conexión:**
- WiFi para ROS2 (comunicación con portátil)
- Serial/WiFi para micro-ROS (comunicación con ESP32)
- USB para cámara zenital (opcional)

---

### 3. ESP32 (Control de Bajo Nivel)

**Función:** Control directo de motores y actuadores

**Especificaciones:**
- Microcontrolador: ESP32-DevKit o similar
- Framework: PlatformIO + micro-ROS

**Tareas:**
- Recibir comandos Twist (`/cmd_vel`) vía micro-ROS
- Implementar cinemática inversa para ruedas Mecanum
- Control PWM de los 4 motores
- Lectura de encoders (futuro)

**Conexión:**
- Serial o WiFi con Raspberry Pi (micro-ROS)
- GPIO para drivers de motores

---

## Sistema de Locomoción

### Configuración: Ruedas Mecanum (X-Drive)

**Cantidad:** 4 ruedas omni-direccionales Mecanum

**Distribución:**
```
        FRENTE
    FL -------- FR
    |    ↑     |
    |    |     |
    RL -------- RR
       TRASERA
```

- **FL:** Front Left (Delantera Izquierda)
- **FR:** Front Right (Delantera Derecha)
- **RL:** Rear Left (Trasera Izquierda)
- **RR:** Rear Right (Trasera Derecha)

**Características:**
- Movimiento omnidireccional (X, Y, rotación)
- Rodamientos a 45° en las ruedas
- Permite movimiento lateral sin girar

### Cinemática Inversa (X-Drive)

```cpp
// Entradas: vx, vy, wz (velocidad lineal X, Y y angular Z)
fl = vx - vy - wz
fr = vx + vy + wz
rl = vx + vy - wz
rr = vx - vy + wz
```

---

## Motores y Drivers

### Motores DC con Encoders

**Cantidad:** 4 motores

**Especificaciones:**
- Tipo: Motores DC con reductora
- Encoders: Sí (para odometría y control preciso)
- Alimentación: Verificar voltaje (típicamente 6-12V)

### Drivers de Motores: 2x Puente H DFRobot

**Configuración:**
- **Driver 1 (Trasero):** Controla RL y RR
- **Driver 2 (Delantero):** Controla FL y FR

**Conexiones ESP32:**

| Motor | PWM Pin | DIR Pin | Canal PWM |
|-------|---------|---------|-----------|
| RL    | GPIO25  | GPIO26  | Canal 0   |
| RR    | GPIO27  | GPIO14  | Canal 1   |
| FL    | GPIO18  | GPIO19  | Canal 2   |
| FR    | GPIO32  | GPIO33  | Canal 3   |

**Características DFRobot:**
- Control mediante PWM + DIR (dirección)
- DIR=HIGH → Adelante, DIR=LOW → Atrás
- PWM (0-255) controla la velocidad

---

## Sistema de Visión

### Cámara IP (Procesada por Portátil)

**Función:** Visión general del campo y detección de elementos

**Características:**
- Conexión: WiFi/Ethernet
- Resolución: HD (720p o superior recomendado)
- Ubicación: Montada en el robot o externa (verificar)

**Procesamiento:**
- OpenCV para detección de objetos
- Detección de ArUco markers
- Publicación en ROS2 topics

### Cámara Zenital + ArUco (Localización)

**Función:** Localización precisa del robot en el campo

**Configuración:**
- Cámara cenital sobre el campo
- Marcadores ArUco en el robot (ID específico, ej: ID=1)
- Diccionario ArUco: DICT_4X4_50

**Funcionamiento:**
- La cámara detecta el marcador del robot
- Calcula posición (X, Y) y orientación (Yaw)
- Permite navegación precisa

---

## Alimentación

### Consideraciones

**Reglamento Eurobot:**
- Tensión máxima: **48V DC**
- Baterías: LiPo con protección o similares

**Distribución (a confirmar):**
- Batería principal para motores
- Reguladores de voltaje para Raspberry Pi (5V)
- Regulador para ESP32 (5V o 3.3V)

**Seguridad:**
- Interruptor de emergencia accesible
- Protección contra cortocircuitos
- Indicador visual de estado (LED)

---

## Sensores y Actuadores Adicionales

### Actuales
- 4x Motores DC con encoders
- 2x Cámaras (IP + opcional zenital)
- Marcadores ArUco para localización

### Posibles Expansiones Futuras
- Sensores de distancia (ultrasonido, LIDAR)
- Servomotores para manipulación de objetos
- Sensores de color
- IMU (para odometría mejorada)
- SIMA (robot secundario pequeño)

---

## Dimensiones del Robot

**Reglamento Eurobot:**
- Diámetro inicial: **≤ 300mm**
- Altura máxima: **≤ 350mm**

**Dimensiones Reales:**
- A medir y documentar

**Peso:**
- A medir (sin límite reglamentario, pero considerar estabilidad)

---

## Comunicaciones

### Red ROS2

**Dominio ROS2:** Configurar DDS domain ID

**Topics principales:**
- `/camera_node/image_raw` - Imagen raw de cámara
- `/camera_node/aruco_image` - Imagen con ArUcos detectados
- `/zenital/image_raw` - Cámara zenital
- `/zenital/debug` - Debug de localización
- `/cmd_vel` (geometry_msgs/Twist) - Comandos de velocidad

**Nodos:**
- `aruco_detector` - Detección de ArUcos
- `eurobot_controller` - Control de navegación
- `esp32_mecanum` - Nodo micro-ROS en ESP32

### micro-ROS (Raspberry Pi ↔ ESP32)

**Transporte:** Serial (USB) o WiFi

**Configuración Raspberry Pi:**
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

**Configuración ESP32:**
```cpp
set_microros_serial_transports(Serial);
```

---

## Referencias de Código

### Ubicaciones

**Proyecto antiguo (referencia):**
- `/home/maki/Desktop/rpi_casa/src/`
  - `robot_vision/` - Detección ArUco
  - `robot_navigator/` - Control y navegación

**Proyecto ESP32 (micro-ROS):**
- `/home/maki/Documents/PlatformIO/Projects/microros_esp_eurobot/`
  - `src/main.cpp` - Control de motores Mecanum

**Proyecto actual:**
- `~/Desktop/laptop_rpi_esp/src/` - Paquetes ROS2 a migrar

---

## Estado del Proyecto

### Componentes Verificados
- ✅ ESP32 con micro-ROS (código base)
- ✅ Cinemática Mecanum implementada
- ✅ Proyecto ROS2 antiguo con visión funcional

### Componentes a Verificar
- ⚠️ Conexión micro-ROS ESP32 ↔ RPI4
- ⚠️ Drivers de motores y cableado
- ⚠️ Integración portátil-RPI4
- ⚠️ Calibración de cámaras
- ⚠️ Encoders de motores (lectura)

### Por Implementar
- ❌ Navegación autónoma completa
- ❌ Detección de elementos del juego 2026
- ❌ Estrategia de competición
- ❌ Tests de hardware completo

---

**Última actualización:** Febrero 2026  
**Responsable:** Equipo [Nombre del equipo]
