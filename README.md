# Robot Planar Dibujante (3-DOF)

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
git clone https://github.com/TU_USUARIO/robot-planar-dibujante.git
cd robot-planar-dibujante

# (opcional) entorno virtual
python -m venv .venv
# Linux/macOS:
source .venv/bin/activate
# Windows:
# .venv\Scripts\activate

pip install -r requirements.txt

