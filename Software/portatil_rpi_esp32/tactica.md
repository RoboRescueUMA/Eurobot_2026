## Táctica de juego propuesta

1. **Posición inicial**: el robot parte de unas coordenadas conocidas `(254, 273)` en el campo.
2. **Despeje frontal**: se desplaza hacia un punto intermedio `(250,77)` y luego a `(284,77)` esquivando las cajas que haya delante, para dejar libre la trayectoria de regreso.
3. **Empuje a origen**: tras el rodeo, empuja las cajas hacia la posición final `(278, 157)` y deja la zona despejada.
4. **Búsqueda de candidatas**: comienza una fase de exploración para localizar cajas “candidatas” (IDs 47/36) que pueda recoger.
5. **Recolección con garra**: una vez montada la garra, se aproximará a cada candidata, la tomará y la depositará en otra posición objetivo.

> Nota: esta estrategia puede requerir identificar varias cajas con el mismo ID. El plan detallado aún está abierto y se actualizará cuando se defina cómo elegir y priorizar las candidatas.
