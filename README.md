# Robot Planar Dibujante (3-DOF)

![Python](https://img.shields.io/badge/Python-3.11-blue)
![OS](https://img.shields.io/badge/Ubuntu-20.04-orange)
![License: MIT](https://img.shields.io/badge/License-MIT-green)
![Status](https://img.shields.io/badge/Status-Stable-brightgreen)

Simula un robot planar de 3 grados de libertad que **extrae contornos de una imagen** y genera la **trayectoria** para animar el trazo. Incluye transición suave entre segmentos (plumón arriba/abajo) y cinemática inversa analítica.

> Proyecto de clase (UDLAP). Implementado originalmente en MATLAB y migrado a **Python** para integración futura con robot real. Probado en **Ubuntu 20.04** con **Python 3.11**.

## Demo rápida
![Animación](docs/captura.png)

## Requisitos
- Python 3.11  
- Ubuntu 20.04 (probado)
- Ver dependencias en `requirements.txt`:
  - `roboticstoolbox-python` (>= 1.1.1)
  - `spatialmath-python` (>= 0.10.0)
  - `numpy` (>= 1.26.4)
  - `opencv-python` (>= 4.11.0.86)
  - `matplotlib` (>= 3.7.4)

## Instalación
```bash
# clonar
git clone https://github.com/Baladier/robot-planar-dibujante.git
cd robot-planar-dibujante

# (opcional) entorno virtual
python -m venv .venv
source .venv/bin/activate   # en Linux/macOS
# .venv\Scripts\activate    # en Windows

pip install -r requirements.txt
````

## Uso

Coloca una imagen en la carpeta `data/` y edita la ruta en el script si es necesario:

```bash
python src/RobotPlanarDibujante.py
```

> El robot generará los contornos detectados con Canny y calculará la trayectoria para simular el dibujo del trazo.

## Estructura del repositorio

```
src/    → código principal (cinemática, planeación, animación)
data/   → imágenes de entrada (ejemplos: gato1.jpg, gato2.jpg…)
docs/   → capturas o animaciones generadas
```

## Áreas de mejora y futuras aplicaciones 🧠

### 🔹 Integración con hardware real

El código puede adaptarse a un brazo físico de 3 GDL o a manipuladores tipo SCARA.
Sugerencias:

* Sustituir las funciones de simulación (`plot`, `jtraj`) por comandos de movimiento reales (por ejemplo, `MoveJ`, `MoveL` en URScript o G-code en CNCs).
* Implementar una capa de comunicación:

  * **UART / USB serial** (con Arduino, STM32 o ESP32) para enviar coordenadas `(x, y, θ)`.
  * **Ethernet / TCP** para control de brazos industriales (ej. UR5, Dobot, myCobot).
* Usar **cinemática inversa** ya calculada por el script como referencia para un controlador PID o LQR en los ejes reales.
* Añadir una función de *planificación de velocidad* o suavizado de aceleraciones (s-curve, quintic splines).

### 🔹 Posibles controladores y plataformas

* **Arduino Due o Mega** con drivers tipo A4988 o TMC2209 (para servos o steppers).
* **Raspberry Pi / Jetson Nano** para integrar procesamiento de imagen + movimiento.
* **STM32 Nucleo / ESP32** si se busca bajo costo y buena velocidad serial.
* **UR5 (Universal Robots)** o similar si se desea conexión directa vía `roboticstoolbox` o `RTDE`.

### 🔹 Mejoras de software

* Implementar un **módulo de interpolación cúbica o spline** para trayectorias más suaves.
* Añadir soporte para **diferentes grosores de trazo** o intensidad según el gradiente de la imagen.
* Integrar un **modo de exportación a CSV o G-code**, para que el robot real ejecute la secuencia de puntos sin Python.
* Añadir un **modo de vista 3D** usando `matplotlib.animation` o `pyvista`.

### 🔹 Posible flujo con microcontrolador (ejemplo)

```
Python (este código)
│
├─ Genera puntos (x, y, θ)
│
└─ Envía por Serial a → Arduino / ESP32
       ↓
       Convierte a PWM / pasos motores
       ↓
       Mueve los ejes (X, Y, θ)
```

## Créditos

Autor: **Alan Beltrán**
Basado en el trabajo colaborativo del equipo de robótica planar (UDLAP).
Licencia: [MIT](LICENSE)

---

```
