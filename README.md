# Planar Drawing Robot (3-DOF)

> 📘 For the Spanish version, see [README_es.md](README_es.md)

![Python](https://img.shields.io/badge/Python-3.11-blue)
![OS](https://img.shields.io/badge/Ubuntu-20.04-orange)
![License: MIT](https://img.shields.io/badge/License-MIT-green)
![Status](https://img.shields.io/badge/Status-Stable-brightgreen)

Simulates a 3-DOF planar robot that **extracts the contours of an image** and generates a **trajectory** to animate the drawing process.  
Includes smooth transitions between segments (pen-up/pen-down) and an analytical inverse kinematics model.

> Class project (UDLAP). Originally implemented in MATLAB and later migrated to **Python** for potential integration with a real robot.  
> Tested on **Ubuntu 20.04** using **Python 3.11**.

## For Instructors
- Tested on Ubuntu 20.04 + Python 3.11.
- Includes example images (`/data`) and screenshots (`/docs`).
- Suggested assignment: extend the code to export CSV/G-code or implement cubic-spline interpolation.

## Quick Demo
![Animation](docs/captura.png)

## Requirements
- Python 3.11  
- Ubuntu 20.04 (tested)
- See dependencies in `requirements.txt`:
  - `roboticstoolbox-python` (>= 1.1.1)
  - `spatialmath-python` (>= 0.10.0)
  - `numpy` (>= 1.26.4)
  - `opencv-python` (>= 4.11.0.86)
  - `matplotlib` (>= 3.7.4)

## Installation
```bash
# clone
git clone https://github.com/Baladier/robot-planar-dibujante.git
cd robot-planar-dibujante

# (optional) virtual environment
python -m venv .venv
source .venv/bin/activate   # Linux/macOS
# .venv\Scripts\activate    # Windows

pip install -r requirements.txt
````

## Usage

Place an image inside the `data/` folder and edit the path in the script if necessary:

```bash
python src/RobotPlanarDibujante.py
```

> The robot will detect image contours using the Canny method and compute the trajectory to simulate the drawing process.

## Repository Structure

```
src/    → main code (kinematics, planning, animation)
data/   → input images (examples: gato1.jpg, gato2.jpg…)
docs/   → screenshots or generated animations
```

## Improvements and Future Applications 🧠

### 🔹 Integration with Real Hardware

The code can be adapted for a physical 3-DOF arm or SCARA-type manipulators.
Suggestions:

* Replace simulation functions (`plot`, `jtraj`) with actual motion commands (e.g., `MoveJ`, `MoveL` in URScript or G-code for CNC systems).
* Implement a communication layer:

  * **UART / USB serial** (Arduino, STM32, or ESP32) to transmit `(x, y, θ)` coordinates.
  * **Ethernet / TCP** for industrial arms (e.g., UR5, Dobot, myCobot).
* Use the **inverse kinematics results** from this script as reference targets for PID or LQR controllers on real axes.
* Add a **velocity-planning** or acceleration-smoothing module (s-curve, quintic splines).

### 🔹 Possible Controllers and Platforms

* **Arduino Due / Mega** with A4988 or TMC2209 drivers (for servos or steppers).
* **Raspberry Pi / Jetson Nano** to combine image processing and motion control.
* **STM32 Nucleo / ESP32** for low-cost systems with high serial speed.
* **UR5 (Universal Robots)** or similar arms for direct connection via `roboticstoolbox` or `RTDE`.

### 🔹 Software Improvements

* Implement a **cubic-spline interpolation module** for smoother paths.
* Add support for **variable stroke thickness** or intensity based on image gradients.
* Include an **export mode to CSV or G-code**, allowing real robots to execute the trajectory offline.
* Integrate a **3D visualization mode** using `matplotlib.animation` or `pyvista`.

### 🔹 Example Microcontroller Workflow

```
Python (this code)
│
├─ Generates (x, y, θ) points
│
└─ Sends via Serial → Arduino / ESP32
       ↓
       Converts to PWM / step signals
       ↓
       Moves axes (X, Y, θ)
```

## Credits

Author: **Alan Beltrán**
Based on the collaborative work of the planar-robotics team (UDLAP).
License: [MIT](LICENSE)

## Contact

For questions or comments about the code:

* 📧 **[alanbeltran1202@gmail.com](mailto:alanbeltran1202@gmail.com)**
* 📧 **[alan.beltrandn@udlap.mx](mailto:alan.beltrandn@udlap.mx)**

You may also open *issues* or suggestions directly in this repository.

---

```
