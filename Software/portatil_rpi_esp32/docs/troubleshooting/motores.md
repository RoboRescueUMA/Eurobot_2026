# Troubleshooting - Motores (Robot Casa - L298N)

## Motor no retrocede

### 2026-02-07: GPIO 16/17 son UART2

**Síntoma:** Motor avanza pero no retrocede (zumba sin moverse)

**Causa:** GPIO 16/17 son pines UART2 (RX2/TX2) con restricciones hardware

**Solución:** Cambio de pines
```cpp
// ANTES
#define M3_IN3 16 // RX2
#define M3_IN4 17 // TX2

// DESPUÉS
#define M3_IN3 18 // GPIO propósito general
#define M3_IN4 19 // GPIO propósito general
```

**Resultado:** ✅ Motor funciona en ambas direcciones. GPIOs 18/19 libres de restricciones.

**Acción:** Reconectar cables + recompilar firmware

---

## Motores zumban pero no se mueven (L298N)

### 2026-02-07: PWM insuficiente por caída de voltaje

**Síntoma:** Con velocidad 0.3 motores zumban pero no se mueven

**Causa:** L298N tiene caída de voltaje ~2V. PWM bajo no genera torque suficiente.

**Solución:** Usar velocidades mínimas 0.8-1.0 (80-100%)

**Resultado:** ✅ Movimiento correcto con velocidades altas

---

## Referencia - Drivers de Motores

**L298N:** 
- Requiere velocidades >0.8 (80%)
- Caída de voltaje ~2V
- Mejor para robots grandes con baterías de alto voltaje

**DFRobot:** 
- Funciona con velocidades >0.3 (30%)
- Menor caída de voltaje
- Mejor para robots pequeños

**Última actualización:** 2026-02-24
