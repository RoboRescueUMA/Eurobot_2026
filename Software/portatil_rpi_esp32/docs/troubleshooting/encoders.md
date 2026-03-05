# Troubleshooting - Encoders

## Todos los encoders muestran 0.0 RPM

### 2026-02-24: Alimentación o conexión incorrecta

**Síntoma:** Todos los encoders reportan 0.0 incluso cuando el robot se mueve

**Causas posibles:**
1. Alimentación de encoders no conectada (VCC y GND)
2. Cables A y B no conectados a GPIO correctos
3. micro-ROS agent no conectado
4. ROS_DOMAIN_ID incorrecto

**Checklist de verificación:**
- [ ] Alimentación de encoders: VCC → 3.3V/5V, GND → GND
- [ ] Cables A y B conectados según `docs/hardware/CONEXION_ENCODERS.md`
- [ ] Monitor serial ESP32 muestra "✅ Encoders configurados"
- [ ] micro-ROS agent conectado sin errores
- [ ] `export ROS_DOMAIN_ID=17` en todos los terminales

**Debug:**
```bash
# Verificar monitor serial
pio device monitor
# Debería mostrar: Encoders RPM -> FL:X.XX FR:X.XX ...

# Girar cada rueda manualmente (con motores desconectados)
# El valor de esa rueda específica debe cambiar
```

**Solución:**
1. Verificar conexiones físicas según guía
2. Asegurar que ROS_DOMAIN_ID=17 está configurado
3. Reiniciar ESP32 después de conectar agent

---

## Un encoder específico siempre 0.0

### 2026-02-24: Cable suelto o GPIO incorrecto

**Síntoma:** Un encoder reporta 0.0 mientras los demás funcionan

**Causas probables:**
1. Cable A o B suelto/roto
2. Alimentación del encoder (VCC/GND) mal conectada
3. GPIO incorrecto en el código

**Solución:**

1. **Verificar pines en código:**
   ```cpp
   // En esp32_roborescue/src/main.cpp
   #define ENCODER_FL_A 34
   #define ENCODER_FL_B 35
   #define ENCODER_FR_A 21
   #define ENCODER_FR_B 22
   #define ENCODER_RL_A 23
   #define ENCODER_RL_B 4
   #define ENCODER_RR_A 16
   #define ENCODER_RR_B 17
   ```

2. **Test rápido:**
   - Intercambiar temporalmente cables A/B de un encoder que funciona con el que no
   - Si el problema se mueve → cable/conector defectuoso
   - Si el problema persiste → conexión GPIO o encoder dañado

---

## Valores muy diferentes entre ruedas

### 2026-02-24: Fricción mecánica desigual

**Síntoma:** Durante avance recto, FL=50 RPM pero FR=10 RPM

**Causas:**
1. Fricción mecánica desigual (ruedas con más resistencia)
2. Motores desbalanceados (algunos más débiles)
3. Voltaje de batería bajo (PWM insuficiente)
4. Encoder mal calibrado (PPR incorrecto)

**Solución temporal:**
- Verificar que todas las ruedas giran libremente
- Cargar batería completamente
- Verificar que PPR en código es correcto (1496 counts/rev)

**Solución a futuro:**
- Implementar control PID individual por rueda
- Usar encoders para ajustar PWM y compensar diferencias

---

## Encoder cuenta en idle (robot detenido pero RPM ≠ 0)

### 2026-02-24: Ruido electromagnético

**Síntoma:** Encoder reporta valores aleatorios cuando robot está detenido

**Causas:**
1. **Ruido electromagnético:** Motores cercanos inducen señales falsas
2. **Cables muy largos:** Actúan como antenas
3. **Pull-ups insuficientes:** Señales flotando
4. **Vibración mecánica:** Robot vibra levemente

**Soluciones:**

1. **Añadir condensadores de desacoplamiento:**
   - 100nF (0.1µF) entre cada cable A/B y GND
   - Soldar lo más cerca posible del conector del encoder

2. **Separación física de cables:**
   - Mantener cables de encoders alejados de cables de potencia
   - Usar cable apantallado si es posible
   - Conectar malla a GND

3. **Añadir resistencias pull-up:**
   - Si encoders son open-collector: 10kΩ entre A/B y VCC

4. **Filtro por software:**
   - Ignorar valores < 5 RPM en el código

---

## RPM negativos cuando deberían ser positivos

### 2026-02-24: Cables A y B intercambiados

**Síntoma:** Encoder cuenta en dirección inversa (signo incorrecto)

**Causa:** Cables A y B intercambiados o motor girando en dirección opuesta

**Solución (recomendada):**
- Intercambiar físicamente cables A y B de ese encoder

**Solución alternativa:**
- Multiplicar por -1 en el código (`update_encoder_velocities()`)

---

## Topic /roborescue/encoder_velocities no aparece

### 2026-02-24: Paquetes ROS no compilados en ESP32

**Síntoma:** Agent conecta pero topic no existe

**Causas posibles:**
1. ESP32 no compilado con paquetes `std_msgs` y `geometry_msgs`
2. Namespace incorrecto en el código
3. Publisher no detectado por agent

**Solución:**
```bash
# Verificar que existe extra_packages.txt
cat esp32_roborescue/extra_packages.txt
# Debe contener: geometry_msgs y std_msgs

# Recompilar librerías
cd esp32_roborescue
pio run --target clean
pio run --target upload

# Reiniciar ESP32 (botón RST)
# Relanzar agent
export ROS_DOMAIN_ID=17
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

---

## Referencia Rápida - Encoders RoboRescue

**Especificaciones:**
- PPR motor: 11 pulsos/revolución
- Reducción: 1:34
- PPR rueda: 374 pulsos/vuelta
- Cuadratura (×4): 1496 counts/rev

**Pines asignados:**
| Motor | Canal A | Canal B |
|-------|---------|---------|
| FL    | GPIO 34 | GPIO 35 |
| FR    | GPIO 21 | GPIO 22 |
| RL    | GPIO 23 | GPIO 4  |
| RR    | GPIO 16 | GPIO 17 |

**Última actualización:** 2026-02-24
