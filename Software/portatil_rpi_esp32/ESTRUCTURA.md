# Estructura del Proyecto Eurobot 2026 - RoboRescue

```
laptop_rpi_esp/
│
├── README.md                           # 📘 Documentación principal
├── .gitignore                          # 🚫 Archivos ignorados por Git
│
├── docs/                               # 📚 DOCUMENTACIÓN
│   ├── reglamento/
│   │   ├── EurobotSenior_general.pdf   # Reglamento oficial completo
│   │   ├── Arena y puntuaciones.pdf    # Detalles del campo 2026
│   │   └── RESUMEN_REGLAMENTO.md       # ⭐ Resumen en español
│   ├── hardware/
│   │   ├── ESPECIFICACIONES_HARDWARE.md # ⭐ Hardware del robot
│   │   └── COMPARATIVA_ROBOTS.md        # ⭐ Diferencias RoboRescue vs Casa
│   └── architecture/
│       └── ARQUITECTURA_SISTEMA.md      # ⭐ Diagramas del sistema
│
├── src/                                # 🤖 PAQUETES ROS2
│   ├── robot_localization/             # 📍 Localización + Navegación (NUEVO)
│   │   ├── robot_localization/
│   │   │   ├── camera_publisher.py     # Captura cámara IP
│   │   │   ├── field_localizer.py      # Homografía + poses absolutas (NUEVO)
│   │   │   └── aruco_navigator.py      # Navegación (por implementar)
│   │   ├── launch/
│   │   │   └── robot_localization.launch.py
│   │   ├── config/
│   │   │   └── robot_localization.yaml
│   │   ├── package.xml
│   │   └── setup.py
│   ├── laptop_vision/                  # 👁️ Visión relativa (ANTIGUO)
│   │   ├── launch/
│   │   ├── config/
│   │   ├── package.xml
│   │   └── setup.py
│   ├── rpi_relay/                      # 🔗 Relay de comandos (RPI4)
│   │   ├── rpi_relay/
│   │   │   └── cmd_vel_relay.py
│   │   ├── package.xml
│   │   └── setup.py
│   ├── robot_vision/                   # (ANTIGUO - Backup)
│   │   └── ...
│   └── robot_navigator/                # (ANTIGUO - Backup)
│       └── ...
│
├── esp32_roborescue/                   # 🏆 ROBOT COMPETICIÓN (DFRobot)
│   ├── README.md                       # ⭐ Documentación específica
│   ├── platformio.ini                  # Configuración PlatformIO
│   ├── src/
│   │   └── main.cpp                    # Control Mecanum PWM+DIR
│   ├── include/
│   ├── lib/
│   └── test/
│
├── esp32_casa/                         # 🏠 ROBOT PRUEBAS (L298N)
│   ├── README.md                       # ⭐ Documentación específica
│   ├── platformio.ini                  # Configuración PlatformIO
│   ├── src/
│   │   └── main.cpp                    # Control Mecanum IN1/IN2+EN
│   ├── include/
│   ├── lib/
│   └── test/
│
├── config/                             # ⚙️ Configuraciones
├── launch/                             # 🚀 Launch files ROS2
│
├── build/                              # 🔨 Compilación ROS2 (ignorado)
├── install/                            # 📦 Instalación ROS2 (ignorado)
└── log/                                # 📝 Logs ROS2 (ignorado)
```

---

## Archivos Clave por Tarea

### 🎯 Para empezar
- `README.md` - Lee esto primero

### 📖 Entender el proyecto
- `docs/reglamento/RESUMEN_REGLAMENTO.md` - Reglas de la competición
- `docs/hardware/ESPECIFICACIONES_HARDWARE.md` - Hardware del robot
- `docs/architecture/ARQUITECTURA_SISTEMA.md` - Cómo funciona todo

### 📍 Nuevo Sistema de Localización (Actual)
- `docs/guias/GUIA_ROBOT_LOCALIZATION.md` - Guía de homografía y localización absoluta
- `src/robot_localization/` - Paquete de localización con poses absolutas
- `src/robot_localization/robot_localization/field_localizer.py` - Nodo principal

### 👁️ Sistema Anterior (Referencia)
- `docs/guias/GUIA_VISION_DISTRIBUIDA.md` - Guía del sistema de visión relativa
- `src/laptop_vision/` - Código anterior

### 🏆 Trabajar con robot de competición
- `esp32_roborescue/README.md` - Instrucciones específicas
- `esp32_roborescue/src/main.cpp` - Código ESP32 DFRobot

### 🏠 Trabajar con robot de pruebas
- `esp32_casa/README.md` - Instrucciones específicas
- `esp32_casa/src/main.cpp` - Código ESP32 L298N

### 🔍 Comparar robots
- `docs/hardware/COMPARATIVA_ROBOTS.md` - Diferencias detalladas

---

## Comandos Rápidos

### Compilar ROS2
```bash
cd ~/Desktop/laptop_rpi_esp
colcon build
source install/setup.bash
```

### Flashear Robot Competición
```bash
cd ~/Desktop/laptop_rpi_esp/esp32_roborescue
pio run --target upload
```

### Flashear Robot Casa
```bash
cd ~/Desktop/laptop_rpi_esp/esp32_casa
pio run --target upload
```

### Iniciar Git
```bash
cd ~/Desktop/laptop_rpi_esp
git init
git add .
git commit -m "Initial commit: Proyecto Eurobot 2026 - RoboRescue"
```

---

**Equipo:** RoboRescue  
**Competición:** Eurobot 2026 Senior  
**Última actualización:** Febrero 2026
