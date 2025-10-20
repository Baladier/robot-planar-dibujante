"""
Robot Planar Dibujante
Autor: Alan Beltrán (Estudiante de Ingeniería Mecatrónica, UDLAP)
Descripción: Este código simula un robot planar 3DOF a partir de una imagen cargada, extrayendo contornos y generando la trayectoria para su animación.

Dependencias:
  - Ubuntu 20.04
  - Python 3.11
  - roboticstoolbox-python >= 1.1.1
  - spatialmath-python >= 0.10.0
  - numpy >= 1.26.4
  - opencv-python >= 4.11.0.86
  - matplotlib >= 3.7.4

"""
import numpy as np
import cv2
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import roboticstoolbox as rtb
import spatialmath as sm

# ====================== PARÁMETROS AJUSTABLES ======================
workspace = 4.0       # Tamaño del área de trabajo en unidades homogéneas
l1, l2, l3 = 2.0, 2.0, 1.0  # Longitudes de los tres eslabones
subfactor = 5         # Submuestreo de puntos por contorno (menor = más puntos)
trans_steps = 50      # Pasos de transición suave entre segmentos
thresh_px = 10        # Umbral en píxeles para separar segmentos discontinuos

# ===================== PARTE 1: PROCESAMIENTO DE IMAGEN =====================
# Cargar imagen en escala de grises y redimensionar
img = cv2.imread('gato1.jpg', cv2.IMREAD_GRAYSCALE)
if img is None:
    raise FileNotFoundError("Imagen no encontrada.")
img = cv2.resize(img, (300, 300))

# Aplicar desenfoque Gaussiano ligero para reducir ruido
gray = cv2.GaussianBlur(img, (5, 5), 0.3)

# Detección de bordes con Canny
edges_canny = cv2.Canny(gray, 100, 200)

# Detección de bordes con Sobel (magnitud)
sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
mag = np.hypot(sobelx, sobely)
_, edges_sobel = cv2.threshold((mag/np.max(mag)*255).astype(np.uint8), 50, 255, cv2.THRESH_BINARY)

# Combinar ambos resultados de detección de bordes
combined = cv2.bitwise_or(edges_canny, edges_sobel)

# Filtrar componentes pequeños para eliminar ruido residual
num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(combined)
filtered = np.zeros_like(combined)
for i in range(1, num_labels):  # Saltar índice 0 (fondo)
    if stats[i, cv2.CC_STAT_AREA] >= 30:
        filtered[labels == i] = 255

# =================== PARTE 2: EXTRACCIÓN Y ORIENTACIÓN CORRECTA ===================
# Extraer contornos externos sin aproximación (todos los píxeles de borde)
contours, _ = cv2.findContours(filtered, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

# Convertir cada punto de contorno a coordenadas (fila, columna)
pixel_points = []
for cnt in contours:
    for p in cnt.squeeze():
        x, y = p[0], p[1]  # OpenCV devuelve (x, y)
        pixel_points.append([y, x])  # [fila, columna]
pixel_points = np.array(pixel_points)

# Corregir la orientación vertical (el origen de la imagen está en la esquina superior izquierda)
max_row = pixel_points[:, 0].max()
raw = []
for row, col in pixel_points:
    row_flipped = max_row - row  # Invertir eje Y para quedar al derecho
    raw.append([row_flipped, col])
raw = np.array(raw)

# Calcular rangos en píxeles para escalar al workspace
min_r, max_r = raw[:,0].min(), raw[:,0].max()
min_c, max_c = raw[:,1].min(), raw[:,1].max()

# =================== PARTE 3: SEPARACIÓN EN SEGMENTOS ===================
segments = []
seg = [raw[0]]
for i in range(1, len(raw)):
    if np.linalg.norm(raw[i] - raw[i-1]) > thresh_px:
        segments.append(np.array(seg))
        seg = [raw[i]]
    else:
        seg.append(raw[i])
segments.append(np.array(seg))

# =================== PARTE 4: CÁLCULO DE IK Y GENERACIÓN DE TRAYECTORIA ===================
Qtotal, PenStates = [], []
q_prev = np.zeros(3)
# Factor de escala para llevar píxeles al workspace (centrado en [2,2])
scale = workspace / max(max_c-min_c, max_r-min_r)
off_x = 2 - (max_c - min_c)*scale/2
off_y = 2 - (max_r - min_r)*scale/2

for s, seg in enumerate(segments):
    # Submuestreo de índices para suavidad
    idxs = list(range(0, len(seg), subfactor))
    if idxs[-1] != len(seg)-1:
        idxs.append(len(seg)-1)

    # Transición suave desde el segmento anterior
    if s > 0:
        r, c = seg[idxs[0]]
        x = (c - min_c)*scale + off_x
        y = (r - min_r)*scale + off_y
        # IK analítico para postura inicial del segmento
        phi = np.arctan2(y, x)
        x2 = x - l3*np.cos(phi)
        y2 = y - l3*np.sin(phi)
        r2 = x2**2 + y2**2
        cos2 = np.clip((r2 - l1**2 - l2**2)/(2*l1*l2), -1, 1)
        t2 = np.arccos(cos2)
        t1 = np.arctan2(y2, x2) - np.arctan2(l2*np.sin(t2), l1 + l2*np.cos(t2))
        t3 = phi - t1 - t2
        q_start = np.array([t1, t2, t3])
        # Interpolación entre q_prev y q_start
        traj = rtb.jtraj(q_prev, q_start, trans_steps)
        for q in traj.q:
            Qtotal.append(q)
            PenStates.append(0)  # plumón arriba durante transición
        q_prev = q_start

    # Recorrido del propio segmento
    for j, idx in enumerate(idxs):
        r, c = seg[idx]
        x = (c - min_c)*scale + off_x
        y = (r - min_r)*scale + off_y
        phi = np.arctan2(y, x)
        x2 = x - l3*np.cos(phi)
        y2 = y - l3*np.sin(phi)
        r2 = x2**2 + y2**2
        cos2 = np.clip((r2 - l1**2 - l2**2)/(2*l1*l2), -1, 1)
        t2 = np.arccos(cos2)
        t1 = np.arctan2(y2, x2) - np.arctan2(l2*np.sin(t2), l1 + l2*np.cos(t2))
        t3 = phi - t1 - t2
        q = np.array([t1, t2, t3])
        Qtotal.append(q)
        PenStates.append(0 if j == 0 else 1)  # plumón abajo tras el primer punto
        q_prev = q

Qtotal = np.array(Qtotal)
PenStates = np.array(PenStates)
print(f"Total steps: {len(Qtotal)}, drawn steps: {PenStates.sum()}")

# =================== PARTE 5: CONFIGURAR ROBOT PARA ANIMACIÓN ===================
L1 = rtb.RevoluteDH(a=l1)
L2 = rtb.RevoluteDH(a=l2)
L3 = rtb.RevoluteDH(a=l3)
robot_planar = rtb.DHRobot([L1, L2, L3], name='Planar3DOF')

# Función que devuelve las posiciones XY de todas las articulaciones
def get_xy(q):
    pts = [np.zeros(2)]
    T = sm.SE3()
    for i, qi in enumerate(q):
        T = T * robot_planar.links[i].A(qi)
        pts.append(T.t[:2])
    return np.array(pts)

# =================== PARTE 6: ANIMACIÓN CON MATPLOTLIB ===================
fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.set_xlim(0, workspace)
ax.set_ylim(0, workspace)
ax.set_title('Robot Planar Dibujando')
robot_line, = ax.plot([], [], '-o', lw=2)
path_line, = ax.plot([], [], 'r-', lw=1)
path_x, path_y = [], []

def init():
    # Inicializar datos vacíos
    robot_line.set_data([], [])
    path_line.set_data([], [])
    return robot_line, path_line

def update(i):
    # Actualizar postura del robot y trazo si el plumón está abajo
    q = Qtotal[i]
    pts = get_xy(q)
    robot_line.set_data(pts[:,0], pts[:,1])
    if PenStates[i] == 1:
        path_x.append(pts[-1,0])
        path_y.append(pts[-1,1])
    else:
        # Insertar NaN para romper la línea y evitar saltos visuales
        path_x.append(np.nan)
        path_y.append(np.nan)
    path_line.set_data(path_x, path_y)
    return robot_line, path_line

# Crear animación sin bucle (repeat=False)
ani = FuncAnimation(
    fig, update, frames=len(Qtotal), init_func=init,
    interval=20, blit=True, repeat=False
)

plt.show()
