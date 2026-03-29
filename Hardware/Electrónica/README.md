# 🤖 Eurobot 2026 — Arquitectura Electrónica

**Diseño eléctrico, esquemas y documentación técnica** del sistema electrónico del robot desarrollado para la competición **Eurobot 2026**.

---

## 🧠 Arquitectura del Sistema

El robot implementa una **arquitectura híbrida de procesamiento**, donde la inteligencia de alto nivel y el control en tiempo real están separados para lograr robustez, velocidad y modularidad:

### 🔹 Nivel Superior — Visión & Estrategia

* **Raspberry Pi**
* Ejecuta:

  * Visión artificial
  * Planificación de alto nivel
  * Lógica de misión
  * Comunicación con sistemas de percepción

### 🔹 Nivel de Control — Tiempo Real & Actuación

* **ESP32**
* Responsable de:

  * Control de motores
  * Control de servos
  * Lectura de sensores
  * Odometría

### 🔗 Comunicación entre controladores

* Conexión **Serial / UART (TX-RX) por cable**
* Permite:

  * Baja latencia
  * Mayor robustez ante interferencias
  * Sin dependencias de red inalámbrica

---

## 🛠️ Componentes Principales

| Ítem | Componente              | Cantidad | Función                        |
| ---- | ----------------------- | -------- | ------------------------------ |
| 1    | ESP32                   | 1        | Control en tiempo real         |
| 2    | Raspberry Pi 5          | 1        | Visión y lógica de misión      |
| 3    | Driver Puente H DRI0002 | 2        | Control de motores DC          |
| 4    | Motor DC + Encoder      | 4        | Tracción y odometría           |
| 5    | Regulador S13V30F5      | 1        | Conversión 12V → 5V            |
| 6–8  | Servomotores            | 3        | Garra y mecanismo de elevación |
| 10   | Seta de emergencia      | 1        | Parada inmediata del sistema   |

---

## 📐 Esquema Eléctrico

El diseño se ha realizado en **KiCad 9.0.6**.

* Entrada principal desde batería **12V nominal (≈11.6V real)**

---

## 🔌 Asignación de Pines — ESP32

### 🏎️ Tracción (Drivers)

| Pines ESP32 | Función   | Motor / Driver            |
| ----------- | --------- | ------------------------- |
| 27 / 14     | PWM / DIR | Delantero Izquierdo (DR1) |
| 25 / 26     | PWM / DIR | Delantero Derecho (DR1)   |
| 32 / 33     | PWM / DIR | Trasero Izquierdo (DR2)   |
| 18 / 19     | PWM / DIR | Trasero Derecho (DR2)     |

---

### 📊 Odometría (Encoders)

| Pines ESP32 | Canal | Encoder             |
| ----------- | ----- | ------------------- |
| 21 / 22     | A / B | Delantero Izquierdo |
| 34 / 35     | A / B | Delantero Derecho   |
| 16 / 17     | A / B | Trasero Izquierdo   |
| 23 / 4      | A / B | Trasero Derecho     |

---

### 🦾 Manipulación (Servos)

| Pin ESP32 | Función | Elemento                   |
| --------- | ------- | -------------------------- |
| ND        | PWM     | Servo Garra                |
| ND        | PWM     | Servo Giro Palma           |
| ND        | PWM     | Servo Elevación            |
| No Usado  | Input   | Encoder Feedback Elevación |

---

## 🛡️ Seguridad y Operación

* 🛑 **Parada de emergencia** mediante seta física que corta alimentación general.
* 🧵 **Arranque con cuerda** para iniciar el ciclo de competición.

---

> ⚡ Este repositorio es parte del desarrollo del robot para la competición Eurobot 2026.
