# Troubleshooting - ESP32

## Error "Failed to communicate with flash chip"

### 2026-02-24: GPIO 12 y 15 interfieren durante boot

**Síntoma:** No se puede subir código al ESP32 con robot conectado

**Causa:** GPIO 12 y 15 son sensibles durante boot del ESP32:
- **GPIO 12:** Controla voltaje de flash (3.3V vs 1.8V)
- **GPIO 15:** Habilita/deshabilita mensajes de debug durante boot

**Solución implementada:**
- Evitar GPIO 12 y 15 para encoders
- Usar GPIO 21, 22, 23 en su lugar (seguros durante boot)

**Si persiste el problema:**
1. Desconectar alimentación del robot (batería/fuente)
2. Mantener solo USB conectado
3. Subir código: `pio run --target upload`
4. Volver a conectar alimentación del robot

---

## Referencia Rápida - GPIOs ESP32

**❌ Evitar:**
- GPIO 0, 2, 15 (boot)
- GPIO 1, 3 (Serial)
- GPIO 6-11 (Flash)
- GPIO 12 (controla voltaje flash - evitar para encoders)
- GPIO 16, 17 (UART2 - problemas con motores)

**✅ Recomendados:**
- GPIO 18, 19, 21, 22, 23
- GPIO 25, 26, 27, 32, 33
- GPIO 4, 5, 13, 14
- GPIO 34, 35 (solo input - ideales para encoders)

**Última actualización:** 2026-02-24
