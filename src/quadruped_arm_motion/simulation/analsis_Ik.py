#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script: ik_por_csv_fail_limits.py

Descripción:
    1) Lee un archivo 'datos_tag.csv' con posiciones (x, y, z) de objetivos,
       resuelve la cinemática inversa para cada punto usando Levenberg–Marquardt,
       Gauss–Newton y Newton–Raphson (en ese orden), guarda en arreglos si tuvo éxito,
       qué método lo resolvió y sus ángulos de articulación, y finalmente grafica
       el robot en postura home junto con un scatter de los objetivos:
       en azul los que tuvieron éxito, en rojo los que fallaron.

    2) A partir de la lista de puntos que fallaron IK, calcula los límites
       aproximados en X, Y y Z (mínimo y máximo) y los muestra por consola.
       Además, pinta en 3D la nube de puntos que fallaron.

Requisitos:
    - Python 3.x
    - numpy
    - matplotlib
    - roboticstoolbox-python
    - spatialmath-python
    - Un archivo 'datos_tag.csv' en el mismo directorio, con líneas “x, y, z”
"""

import numpy as np
import matplotlib.pyplot as plt

from roboticstoolbox.robot.DHLink import RevoluteDH
from roboticstoolbox.robot.DHRobot import DHRobot
from spatialmath import SE3

# ------------------------------------------------------
# 1) Cargar datos desde CSV para IK
# ------------------------------------------------------
try:
    data = np.loadtxt(r'C:\Users\fporras\OneDrive - Antonio Matachana S.A\Documents\TFM\datos_tag.csv', delimiter=',')
except Exception as e:
    print(f"Error al leer 'datos_tag.csv': {e}")
    print("Asegúrate de que el archivo exista y tenga formato 'x, y, z' en cada línea.")
    exit(1)

# ------------------------------------------------------
# 2) Construir el robot DH de 5 eslabones
# ------------------------------------------------------
dh_Arm = [
    ("art1", 0.32,   0.00,   np.deg2rad(-180), np.deg2rad( 80.0), (np.deg2rad(-1.5),  np.deg2rad(1.5))),
    ("art2", -0.0800,0.00,   np.deg2rad(  90), np.deg2rad( 55.0), (np.deg2rad(-0.5),  np.deg2rad(55))),
    ("art3", 0.00,   0.30,   np.deg2rad(   0), np.deg2rad(-45.0), (np.deg2rad(-0.5),  np.deg2rad(35))),
    ("art4", 0.00,   0.1,   np.deg2rad(-90),  np.deg2rad(  30.0),(np.deg2rad(-0.5),  np.deg2rad(180))),
    ("art5", 0.3200, 0.00,   np.deg2rad(-90),  np.deg2rad(  0.0), (np.deg2rad(-0.5),  np.deg2rad(180))),
]
links = [
    RevoluteDH(d=d, a=a, alpha=alpha, offset=offset, qlim=bounds)
    for (_, d, a, alpha, offset, bounds) in dh_Arm
]
robot = DHRobot(links, name='arm')

# ------------------------------------------------------
# 3) Definir la postura 'home' (todas las articulaciones en 0)
# ------------------------------------------------------
home = [0.0, 0.0, 0.0, 0.0, 0.0 ]

# ------------------------------------------------------
# 4) Semilla inicial para los solvers de IK
# ------------------------------------------------------
q0 = np.zeros(5)

# ------------------------------------------------------
# 5) Máscara para ignorar rotación (solo se resuelve posición)
# ------------------------------------------------------
mask_translation = [0.3,0.3, 0.2, 0,0]


# ------------------------------------------------------
# 6) Orientación fija para todos los objetivos: rotación en Z de π/2
# ------------------------------------------------------
R_obj = SE3.RPY([0, 0, -np.pi/2], order='xyz')

# ------------------------------------------------------
# 7) Preparar estructuras para guardar resultados de IK
# ------------------------------------------------------
results = []
success_points = []
fail_points = []
joints_solutions = []
data_unique = np.unique(data, axis=0)
print(f"Total de puntos únicos a procesar: {len(data_unique)}")
# ------------------------------------------------------
# 8) Iterar sobre cada fila del CSV y resolver IK
# ------------------------------------------------------
for row in data_unique:
    x, y, z = row
    print(f"Procesando punto: x={x:.3f}, y={y:.3f}, z={z:.3f},row={row}")
    T_target = SE3(x, y, z) * R_obj

    # Intento IK con Levenberg–Marquardt
    q_lm, success_lm, _, _, _ = robot.ik_LM(
        T_target, q0=q0, ilimit=4000, slimit=150, tol=1e-4,
        start = "art1",
        mask=mask_translation, joint_limits=True
    )
    if success_lm:
        results.append({'x': x, 'y': y, 'z': z, 'success': True,  'method': 'LM', 'q': q_lm})
        success_points.append((x, y, z))
        joints_solutions.append(q_lm)
        continue

    # Si LM falla, intentar Gauss–Newton
    q_gn, success_gn, _, _, _ = robot.ik_GN(
        T_target, q0=q0, ilimit=4000, slimit=150, tol=1e-4,
        start = "art1",
        mask=mask_translation, joint_limits=True, pinv=False
    )
    if success_gn:
        results.append({'x': x, 'y': y, 'z': z, 'success': True,  'method': 'GN', 'q': q_gn})
        success_points.append((x, y, z))
        joints_solutions.append(q_gn)
        continue

    # Si GN falla, intentar Newton–Raphson
    q_nr, success_nr, _, _, _ = robot.ik_NR(
        T_target, q0=q0, ilimit=4000, slimit=150, tol=1e-4,
        start = "art1",
        mask=mask_translation, joint_limits=True, pinv=False
    )
    if success_nr:
        results.append({'x': x, 'y': y, 'z': z, 'success': True,  'method': 'NR', 'q': q_nr})
        success_points.append((x, y, z))
        joints_solutions.append(q_nr)
        continue
    else:
        # Ningún método funcionó
        print(f"Fallo IK para punto: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        results.append({'x': x, 'y': y, 'z': z, 'success': False, 'method': None, 'q': None})
        fail_points.append((x, y, z))

# ------------------------------------------------------
# 9) Mostrar resumen de IK
# ------------------------------------------------------
n_total   = len(data_unique)
n_success = sum(r['success'] for r in results)
n_fail    = n_total - n_success

print("==============================================")
print(f"Total de objetivos: {n_total}")
print(f"Éxitos IK: {n_success}")
print(f"Fallos IK: {n_fail}")
print("==============================================\n")

# ------------------------------------------------------
# 10) Calcular límites a partir de los puntos que fallaron IK
# ------------------------------------------------------
if len(fail_points) > 0:
    fail_arr = np.array(fail_points)  # forma (n_fallos, 3)
    x_min_fail, y_min_fail, z_min_fail = np.min(fail_arr, axis=0)
    x_max_fail, y_max_fail, z_max_fail = np.max(fail_arr, axis=0)

    print("Límites aproximados de los puntos que fallaron IK:")
    print(f"  X: [{x_min_fail:.3f}, {x_max_fail:.3f}] metros")
    print(f"  Y: [{y_min_fail:.3f}, {y_max_fail:.3f}] metros")
    print(f"  Z: [{z_min_fail:.3f}, {z_max_fail:.3f}] metros")
    print("==============================================\n")
else:
    print("No hubo puntos que fallaran IK; no se pueden calcular límites.\n")

# ------------------------------------------------------
# 11) Graficar robot en 'home' y scatter de objetivos IK
# ------------------------------------------------------
fig1 = plt.figure(figsize=(8, 6))
robot.plot(home, backend='pyplot', fig=fig1, block=False)
ax1 = plt.gca()

if success_points:
    sp = np.array(success_points)
    ax1.scatter(sp[:, 0], sp[:, 1], sp[:, 2],
                c='blue', s=40, label='Éxitos IK')
if fail_points:
    fp = np.array(fail_points)
    ax1.scatter(fp[:, 0], fp[:, 1], fp[:, 2],
                c='red', s=40, label='Fallos IK')

ax1.set_title("Robot en POSTURA HOME y Puntos de IK")
ax1.set_xlabel("X (m)")
ax1.set_ylabel("Y (m)")
ax1.set_zlabel("Z (m)")
ax1.legend()
sp_array = np.array(success_points)  # forma (n_exitos, 3)
x_vals = sp_array[:, 0]
y_vals = sp_array[:, 1]
z_vals = sp_array[:, 2]

x_min_succ = x_vals.min()
x_max_succ = x_vals.max()
y_min_succ = y_vals.min()
y_max_succ = y_vals.max()
z_min_succ = z_vals.min()
z_max_succ = z_vals.max()

print("==============================================")
print("Límites aproximados de los puntos EXITOSOS (alcanzables):")
print(f"  X mínimo: {x_min_succ:.3f} metros")
print(f"  X máximo: {x_max_succ:.3f} metros")
print(f"  Y mínimo: {y_min_succ:.3f} metros")
print(f"  Y máximo: {y_max_succ:.3f} metros")
print(f"  Z mínimo: {z_min_succ:.3f} metros")
print(f"  Z máximo: {z_max_succ:.3f} metros")
print("==============================================")
# ------------------------------------------------------
# 12) Graficar nube de puntos de las posiciones que fallaron IK
# ------------------------------------------------------
if fail_points:
    fig2 = plt.figure(figsize=(8, 6))
    ax2 = fig2.add_subplot(111)
    ax2.scatter(fp[:, 0], fp[:, 1], fp[:, 2],
                c='red', s=5, alpha=0.5)
    ax2.set_title("Nube de puntos que FALLARON IK")
    ax2.set_xlabel("X (m)")
    ax2.set_ylabel("Y (m)")
    ax2.set_zlabel("Z (m)")
else:
    print("No se dibuja la nube de fallos porque no hubo puntos fallidos.")

plt.show()
