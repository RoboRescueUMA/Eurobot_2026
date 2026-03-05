# Troubleshooting - micro-ROS

## micro-ROS: "errno: 16 - Device busy"

### 2026-02-24: ROS_DOMAIN_ID no configurado

**Síntoma:** Agent no conecta, error "Device busy"

**Causa:** ROS_DOMAIN_ID no configurado o diferente entre ESP32 y RPI

**Solución:**
```bash
# Configurar en todos los terminales
export ROS_DOMAIN_ID=17

# Añadir permanentemente a ~/.bashrc (Laptop y RPI)
echo "export ROS_DOMAIN_ID=17" >> ~/.bashrc
source ~/.bashrc
```

**Verificar que está configurado:**
```bash
echo $ROS_DOMAIN_ID
# Debe mostrar: 17
```

---

## Agent conecta pero no hay comunicación

### 2026-02-24: ESP32 no responde después de conectar

**Síntoma:** Agent dice "connected" pero no aparecen topics del ESP32

**Solución:**
1. **Reiniciar ESP32:** Presionar botón RST físicamente
2. **Relanzar agent:**
   ```bash
   # Matar proceso anterior si existe
   pkill -9 micro_ros_agent
   
   # Lanzar nuevamente
   export ROS_DOMAIN_ID=17
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
   ```

3. **Verificar topics:**
   ```bash
   ros2 topic list | grep roborescue
   # Debería aparecer: /roborescue/cmd_vel, /roborescue/encoder_velocities
   ```

---

## Puerto serial no encontrado (/dev/ttyUSB0)

### 2026-02-24: ESP32 no detectado por sistema

**Síntoma:** Error "No such file or directory: /dev/ttyUSB0"

**Solución:**

1. **Verificar qué puerto usa el ESP32:**
   ```bash
   ls /dev/ttyUSB* /dev/ttyACM*
   # Puede ser: /dev/ttyUSB0, /dev/ttyUSB1, /dev/ttyACM0, etc.
   ```

2. **Dar permisos al usuario:**
   ```bash
   sudo usermod -a -G dialout $USER
   # Logout y login para aplicar cambios
   ```

3. **Lanzar agent con puerto correcto:**
   ```bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB1 -b 115200
   ```

---

**Última actualización:** 2026-02-24
