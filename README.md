# 🤖 Eurobot 2026 — RoboRescue Team

Repositorio oficial del equipo **RoboRescue** para la competición **Eurobot 2026**.

---

## 🧠 ¿Qué es Eurobot?

**Eurobot** es una competición internacional de robótica donde equipos diseñan, construyen y programan robots autónomos capaces de realizar tareas en un entorno de juego específico.

Cada año, la competición propone un nuevo desafío con reglas propias, donde los robots deben:
* Actuar de forma completamente autónoma
* Interactuar con elementos del entorno
* Competir contra otro robot en tiempo real
* Maximizar la puntuación en un tiempo limitado

---
## 🧊 Eurobot 2026 — “Winter is Coming”
El objetivo general del juego es ayudar a unas ardillas a recuperar y proteger sus bellotas antes de que vuelvan los humanos, gestionando misiones como recuperar 
cajas, controlar condiciones ambientales o devolverlas a sus nidos antes de tiempo.

* Para más información sobre la competicion leer --> Reglamento / EurobotSenior_General.pdf

---

## 🧩 Estructura del Repositorio

El proyecto está organizado en varios módulos principales:

```
📂 Eurobot_2026
├── ⚙️ hardware/                           # Diseño mecánico y 3D, esquemas electrónicos y slprt de componentes
├── 🧠 software/                           # Código del robot (ROS2, control, visión, etc.)
├── 📄 reglamento/                         # Documentación técnica
└── 📦 documentos de referencia/           # Documentos que pueden ser de ayuda
```

---

## ⚙️ Arquitectura del Sistema

El robot sigue una arquitectura distribuida:

### 🔹 Nivel Alto (Computación)

* Procesamiento de visión
* Planificación de trayectoria
* Toma de decisiones

### 🔹 Nivel Bajo (Control en Tiempo Real)

* Lectura de sensores
* Control de motores

### 🔹 Comunicación

* Middleware basado en **ROS 2**
* Intercambio de datos entre nodos

---

## 🚀 Tecnologías Utilizadas

* **ROS 2** → Comunicación entre nodos
* **Python / C++** → Desarrollo de software
* **ESP32** → Control de hardware
* **OpenCV** → Visión artificial
* **CAD (SolidWorks)** → Diseño mecánico

---

## 🏁 Estado del Proyecto

🚧 En desarrollo — Temporada Eurobot 2026

---
