# <span style="font-size:32px;">🤖 RoboRescue – Nodo de Odometría 4WD (ROS 2)</span>

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue) ![Python](https://img.shields.io/badge/Python-3.10-green)

<p style="font-size:18px;">Este nodo estima la <strong>pose</strong> de nuestro robot omnidireccional de 4 ruedas en tiempo real usando las velocidades de los encoders y la cinemática directa.
Solo necesita un frame de la camara --> No afecta el retardo de la cámara</p>


---

## <span style="font-size:24px;">📡 Qué hace</span>

- Calcula la <span style="color:blue; font-weight:bold;">posición `(x, y)`</span> y <span style="color:green; font-weight:bold;">orientación `θ`</span> del robot
- Convierte velocidades de los encoders en <strong>velocidades lineales y angular</strong>
- Integra el movimiento en el <strong>marco global</strong> con método de Euler

---

## <span style="font-size:24px;">🔌 Suscripciones</span>

| Topic | Tipo | Descripción |
|-------|------|------------|
| `/encoders` | `std_msgs/Float32MultiArray` | Velocidades `[v1, v2, v3, v4]` (ticks/s) |
| `/initial_pose` | `geometry_msgs/Pose2D` | Pose inicial `(x, y, θ)` |

---

## <span style="font-size:24px;">📤 Publicaciones</span>

| Topic | Tipo | Descripción |
|-------|------|------------|
| `/pose2d` | `geometry_msgs/Pose2D` | Pose estimada del robot |

---

## <span style="font-size:24px;">⚙️ Parámetros del robot (m) </span>
- ANCHO = 0.200    
- LARGO = 0.153    
- R = (ANCHO + LARGO) / 2  --> Radio cinemático

## <span style="font-size:24px;"> Encoders y conversión </span>
- DIAMETRO_RUEDA   = 0.06   
- TICKS_POR_VUELTA = 360 --> Cambiar al valor correcto

Conversión de ticks a velocidad lineal:
<span style="color:purple;">v (m/s) = ticks/s × (circunferencia / ticks_por_vuelta)</span>

## <span style="font-size:24px;">🧭 Cinemática</span>

Velocidades en el marco del robot:
- vx_r = (v1 + v2 + v3 + v4)/4
- vy_r = (-v1 + v2 + v3 - v4)/4
- omega = (-v1 + v2 - v3 + v4)/(4*R)

Integración al marco global:
- x += (vx_rcosθ - vy_rsinθ) * dt
- y += (vx_rsinθ + vy_rcosθ) * dt
- θ += omega * dt --> θ normalizado a [-π, π]

<span style="font-size:24px;">🛞 Disposición de ruedas</span>

        FRENTE
    v1 ┌────┐ v2
       │    │
    v4 └────┘ v3
        ATRÁS


<span style="font-size:24px;">⚠️ Notas importantes</span>

- Velocidades de entrada en <span style="color:red; font-weight:bold;">ticks/s</span>
- Ignora dt inválidos (dt ≤ 0 o dt > 1)
- La orientación inicial (θ) se fija a <span style="color:green;">0 rad</span> si no se publica otra (Asumimos que empezamos con una orientacion perfecta)
