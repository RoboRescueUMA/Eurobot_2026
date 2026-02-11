# Troubleshooting - Registro de Problemas

## 🤖 Robot Casa - L298N

### 2026-02-07: Motor 3 no retrocede

**Síntoma:** Motor avanza pero no retrocede (zumba sin moverse)

**Causa:** GPIO 16/17 son pines UART2 (RX2/TX2) con restricciones hardware

**Solución:** Cambio de pines Motor 3
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

### 2026-02-07: L298N requiere velocidades altas

**Síntoma:** Con velocidad 0.3 motores zumban pero no se mueven

**Causa:** L298N tiene caída de voltaje ~2V. PWM bajo no genera torque suficiente.

**Solución:** Usar velocidades mínimas 0.8-1.0 (80-100%)

**Resultado:** ✅ Movimiento correcto con velocidades altas

---

## 📝 Referencia Rápida

### GPIOs ESP32 para motores

**❌ Evitar:**
- GPIO 0, 2, 15 (boot)
- GPIO 1, 3 (Serial)
- GPIO 6-11 (Flash)
- GPIO 16, 17 (UART2)

**✅ Recomendados:**
- GPIO 18, 19, 21, 22, 23
- GPIO 25, 26, 27, 32, 33
- GPIO 4, 5, 12, 13, 14

### Drivers

**L298N:** Requiere velocidades >0.8 (caída 2V)  
**DFRobot:** Funciona con velocidades >0.3 (menor caída)

---

**Última actualización:** 2026-02-07
