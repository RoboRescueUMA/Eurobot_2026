# Troubleshooting - Xbox Controller

## 🎮 Problemas Comunes del Xbox Controller

Este documento cubre problemas específicos del control del robot mediante Xbox Controller.

**Componente:** `xbox_controler/xbox_teleop.py`  
**Referencias:** Ver `docs/guias/GUIA_XBOX_CONTROLLER.md`

---

## ❌ Joystick No Es Detectado

### Síntomas
```
Error: No joystick device found
ls /dev/input/js* returns nothing
jstest-gtk shows "No devices"
```

### Causas y Soluciones

#### 1. **Joystick No Conectado**

**Verificación:**
```bash
# Ver dispositivos USB
lsusb | grep -i joy
lsusb | grep -i "microsoft\|xbox"

# Verificar que aparece en /dev/input/
ls -la /dev/input/js*  # Debería mostrar js0, js1, etc.
```

**Solución:**
- Desconecta y vuelve a conectar el joystick
- Intenta otro puerto USB
- Prueba el joystick en otra computadora para verificar que funciona

---

#### 2. **Permisos Insuficientes**

**Error típico:**
```
Permission denied: /dev/input/js0
```

**Soluciones rápidas:**

```bash
# Opción A: Cambiar permisos temporalmente (pierde efecto en reinicio)
sudo chmod 666 /dev/input/js*

# Opción B: Agregar usuario al grupo dialout (permanente)
sudo usermod -a -G input $USER
# Requiere logout/login para aplicarse

# Verificar que funciona:
jstest /dev/input/js0
```

---

#### 3. **Controlador joy_node No Instalado**

**Verificación:**
```bash
ros2 pkg list | grep joy
# Debería mostrar: joy
```

**Instalación:**
```bash
# Ubuntu/Debian
sudo apt install -y ros-humble-joy

# Luego reinicia/recompila
source /opt/ros/humble/setup.bash
```

---

## 🔕 Joystick Detectado pero Robot No Responde

### Síntomas
- Joystick funciona en otras apps
- `jstest` muestra movimientos del stick
- Pero `/roborescue/cmd_vel` no recibe comandos

### Causas y Soluciones

#### 1. **joy_node No Está Corriendo**

**Verificación:**
```bash
# Ver nodos activos
ros2 node list | grep joy

# Si no aparece, iniciar manualmente:
ros2 run joy joy_node --ros-args -p dev:=/dev/input/js0
```

**Solución automática (agregar a startup):**
```bash
# Crear script de inicio: ~/.bashrc o similar
alias joy_start='ros2 run joy joy_node --ros-args -p dev:=/dev/input/js0'
```

---

#### 2. **xbox_teleop No Está Corriendo**

**Verificación:**
```bash
# Ver nodos activos
ros2 node list | grep xbox

# Si no aparece, iniciar:
ros2 run xbox_controler xbox_teleop
```

**Troubleshooting si da error:**
```bash
# Verificar que el paquete está compilado
colcon build --packages-select xbox_controler
source install/setup.bash

# Luego intentar de nuevo
ros2 run xbox_controler xbox_teleop
```

---

#### 3. **Namespace Incorrecto**

**Problema:** El nodo publica a `/roborescue/cmd_vel` pero el robot espera en otro topic

**Verificación:**
```bash
# Ver a dónde publica el xbox_teleop
ros2 topic info /roborescue/cmd_vel
# Debería mostrar "Type: geometry_msgs/msg/Twist"

# Ver suscriptores
ros2 topic list | grep cmd_vel
# Debería haber un suscriptor en el robot
```

**Solución:**
```bash
# Si necesitas cambiar el topic de destino:
ros2 run xbox_controler xbox_teleop \
    -p cmd_vel_topic:=/roborescue/cmd_vel_custom
```

---

#### 4. **Botón de Seguridad (RB) No Está Presionado**

**Síntoma:**
- El joystick mueve pero el robot no se mueve
- Suelta RB y el robot se detiene

**Solución:**
- Mantén presionado RB (botón 5) mientras mueves los sticks
- Si RB está roto, cambiar el botón de seguridad:

```bash
# Usar botón Y (botón 3) como seguridad
ros2 run xbox_controler xbox_teleop \
    -p enable_button:=3
```

---

## 🔀 Controles Invertidos o Incorrectos

### Síntomas
- Adelante hace retroceder
- Giro es al revés
- Sticks no responden como se espera

### Causas y Soluciones

#### 1. **Ejes Mapeados Incorrectamente**

**Primero, verificar qué ejes corresponden:**
```bash
# Iniciar jstest
jstest /dev/input/js0

# Mover sticks y tomar nota:
# - Stick izquierdo arriba/abajo → observar qué eje cambia
# - Stick izquierdo izq/der → observar qué eje cambia
# - Stick derecho izq/der → observar qué eje cambia
```

**Mapeo estándar Xbox 360/One:**

| Movimiento | Eje | Rango |
|------------|-----|-------|
| Stick Izq. Arriba/Abajo | Eje 1 | -1.0 a 1.0 |
| Stick Izq. Izq/Der | Eje 0 | -1.0 a 1.0 |
| Stick Der. Izq/Der | Eje 3 | -1.0 a 1.0 |
| Stick Der. Arriba/Abajo | Eje 4 | -1.0 a 1.0 |
| LT (Gatillo izq.) | Eje 2 | -1.0 a 1.0 |
| RT (Gatillo der.) | Eje 5 | -1.0 a 1.0 |

**Solución (reconfigurar ejes):**
```bash
# Si tu joystick tiene ejes diferentes:
ros2 run xbox_controler xbox_teleop \
    -p axis_linear_x:=1 \
    -p axis_linear_y:=0 \
    -p axis_angular_yaw:=3
```

---

#### 2. **Direcciones Invertidas (Escalas Negativas)**

**Síntomas:**
- Adelante hace retroceder
- Giro es opuesto
- Strafe es al revés

**Solución (invertir direcciones):**
```bash
# Invertir adelante/atrás:
ros2 run xbox_controler xbox_teleop \
    -p scale_linear_x:=-1.0

# Invertir strafe:
ros2 run xbox_controler xbox_teleop \
    -p scale_linear_y:=1.0  # Cambiar de -1.0 a 1.0

# Invertir giro:
ros2 run xbox_controler xbox_teleop \
    -p scale_angular_yaw:=1.0  # Cambiar de -1.0 a 1.0
```

---

#### 3. **Velocidad Muy Alta/Baja**

**Síntomas:**
- Robot es demasiado sensible (se mueve rápido)
- Robot es muy lento
- Movimientos no son suave

**Soluciones:**

Reducir velocidad:
```bash
ros2 run xbox_controler xbox_teleop \
    -p scale_linear_x:=0.5 \
    -p scale_linear_y:=0.5 \
    -p scale_angular_yaw:=0.3
```

Aumentar velocidad:
```bash
ros2 run xbox_controler xbox_teleop \
    -p scale_linear_x:=1.0 \
    -p scale_linear_y:=1.0 \
    -p scale_angular_yaw:=0.8
```

**Valores recomendados por caso:**

| Caso | scale_linear_x | scale_angular_yaw | Notas |
|------|---|---|---|
| Muy sensible | 0.3-0.5 | 0.2-0.3 | Para espacios reducidos |
| Normal (recomendado) | 0.8-1.0 | 0.5-0.7 | Para competencia |
| Precisión (lento) | 0.2-0.3 | 0.1-0.2 | Para pruebas delicadas |
| Máxima velocidad | 1.0 | 1.0 | Riesgo: puede ser incontrolable |

---

## 📊 Verificación de Funcionamiento

### Test 1: Verificar que joy_node publica

```bash
# Terminal 1: Iniciar joy_node
ros2 run joy joy_node

# Terminal 2: Ver datos del joystick
ros2 topic echo /joy

# Resultado esperado: ver cambios en axes y buttons al mover el control
```

---

### Test 2: Verificar que xbox_teleop procesa inputs

```bash
# Terminal 1: Iniciar joy_node
ros2 run joy joy_node

# Terminal 2: Iniciar xbox_teleop
ros2 run xbox_controler xbox_teleop

# Terminal 3: Monitorear salida
ros2 topic echo /roborescue/cmd_vel

# Resultado esperado:
# - Mover stick izq. arriba → linear.x cambia
# - Mover stick izq. izq → linear.y cambia
# - Mover stick der. izq/der → angular.z cambia
```

---

### Test 3: Verificar que robot recibe comandos

```bash
# Terminal 1: Iniciar joy_node y xbox_teleop (como arriba)

# Terminal 2: Ver información del topic
ros2 topic info /roborescue/cmd_vel

# Resultado esperado:
# "Publishers: 1"  (xbox_teleop)
# "Subscribers: 1" (robot)

# Si hay 0 suscriptores, el robot no está escuchando
```

---

## 🔌 Joystick Desconectado/Reconectado

### Síntoma
Después de desconectar y reconectar el joystick, los comandos no funcionan

### Solución

```bash
# 1. Detener joy_node
ros2 node kill /joy_node

# 2. Esperar 2 segundos

# 3. Reconectar joystick USB

# 4. Reiniciar joy_node
ros2 run joy joy_node

# 5. Verificar que está activo
ros2 node list | grep joy
```

---

## ⚡ Comportamiento Errático (Movimientos Aleatorios)

### Síntomas
- Robot se mueve sin que toque el joystick
- Movimientos inconsistentes
- ROS_DOMAIN_ID conflictivo

### Causas y Soluciones

#### 1. **Interferencia con Otro ROS2 (Domain ID Conflict)**

**Verificación:**
```bash
# Ver ROS_DOMAIN_ID actual
echo $ROS_DOMAIN_ID

# Debería ser: 17
```

**Solución:**
```bash
# En laptop:
export ROS_DOMAIN_ID=17

# En RPI:
export ROS_DOMAIN_ID=17

# Añadir a ~/.bashrc para persistencia:
echo "export ROS_DOMAIN_ID=17" >> ~/.bashrc
source ~/.bashrc
```

---

#### 2. **Múltiples Instancias de xbox_teleop**

**Verificación:**
```bash
# Ver cuántos nodos xbox_teleop hay
ros2 node list | grep xbox

# Si hay más de 1, hay conflicto
```

**Solución:**
```bash
# Matar todas las instancias
pkill -f xbox_teleop

# Iniciar una sola:
ros2 run xbox_controler xbox_teleop
```

---

#### 3. **Joystick Defectuoso (Drift)**

**Síntomas:**
- Sin mover el stick, los ejes varían
- Valor "reposo" no es cero (0.1-0.5)

**Verificación:**
```bash
# Sin tocar el joystick:
jstest /dev/input/js0

# Observar los ejes, todos deberían estar en 0.00
```

**Soluciones:**
1. Calibrar joystick (si el SO proporciona herramienta)
2. Aumentar "dead zone" en código (ignorar pequeños valores):

```python
# Agregar en xbox_teleop.py después de leer los ejes:
DEADZONE = 0.1

if abs(cmd.linear.x) < DEADZONE:
    cmd.linear.x = 0.0
if abs(cmd.linear.y) < DEADZONE:
    cmd.linear.y = 0.0
if abs(cmd.angular.z) < DEADZONE:
    cmd.angular.z = 0.0
```

3. Si no funciona, el joystick puede estar defectuoso → reemplazar

---

## 🖥️ Comandos Útiles para Diagnóstico

```bash
# Lista de comandos útiles para debugging

# 1. Ver dispositivos de entrada disponibles
lsusb | grep -i joy
ls /dev/input/

# 2. Probar joystick directamente
jstest /dev/input/js0
jstest-gtk

# 3. Ver nodos activos
ros2 node list

# 4. Ver topics disponibles
ros2 topic list

# 5. Verificar suscriptor de cmd_vel
ros2 topic info /roborescue/cmd_vel

# 6. Monitorear inputs del joystick
ros2 topic echo /joy

# 7. Monitorear comandos enviados
ros2 topic echo /roborescue/cmd_vel

# 8. Matar nodo específico
ros2 node kill /xbox_teleop
ros2 node kill /joy_node

# 9. Elevar verbosidad de logs
ros2 run xbox_controler xbox_teleop --ros-args --log-level DEBUG

# 10. Ver parámetros del nodo
ros2 param list /xbox_teleop
ros2 param get /xbox_teleop scale_linear_x
```

---

## 📋 Checklist de Diagnóstico

- [ ] Joystick conectado (`lsusb` muestra device)
- [ ] Archivo de dispositivo existe (`/dev/input/js0`)
- [ ] Permisos suficientes (sin "Permission denied")
- [ ] `joy_node` está corriendo (`ros2 node list`)
- [ ] `/joy` topic tiene datos (`ros2 topic echo /joy`)
- [ ] `xbox_teleop` está corriendo
- [ ] `/roborescue/cmd_vel` recibe datos
- [ ] Robot está suscrito a `/roborescue/cmd_vel`
- [ ] ROS_DOMAIN_ID=17 está configurado
- [ ] Botón RB funciona (presionado → robot responde)
- [ ] Joystick no tiene drift (test en `jstest`)

---

## 🔗 Referencias

- **Código principal:** `src/xbox_controler/xbox_controler/xbox_teleop.py`
- **Guía de uso:** `docs/guias/GUIA_XBOX_CONTROLLER.md`
- **ROS2 Joy:** https://github.com/ros-drivers/joystick_drivers
- **Documentación Joy:** https://docs.ros.org/en/humble/p/joy/

---

**Última actualización:** 2026-03-05  
**Equipo:** RoboRescue
