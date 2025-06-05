#!/usr/bin/env python3
import math
import numpy as np
import matplotlib.pyplot as plt

from ikpy.chain import Chain
from ikpy.link import DHLink
import ikpy.utils.geometry as geom

# -----------------------------------------------
# 1) Construir la cadena a partir de los parámetros DH
# -----------------------------------------------
dh_Arm = [
    # nombre, d (m),   a (m),  alpha (rad),            theta_offset (rad),       límites (rad)
    ("art1", 0.32,   0.00,   np.deg2rad(-180), np.deg2rad( 80.0),  (np.deg2rad(-0.5),  np.deg2rad(35))),
    ("art2", -0.080, 0.00,   np.deg2rad(  90), np.deg2rad( 50.0),  (np.deg2rad(-0.5),  np.deg2rad(20))),
    ("art3",  0.00,  0.30,   np.deg2rad(   0), np.deg2rad(-80.0),  (np.deg2rad(-0.5),  np.deg2rad(15))),
    ("art4",  0.00,  0.10,   np.deg2rad(-90),  np.deg2rad( 50.0),  (np.deg2rad(-0.5),  np.deg2rad(180))),
    ("art5",  0.320, 0.00,   np.deg2rad(-90),  np.deg2rad(  0.0),  (np.deg2rad(-0.5),  np.deg2rad(180))),
]

def build_arm_chain_from_dh():
    links = []
    # Si quieres añadir un eslabón de origen fijo (OriginLink), lo activas aquí:
    # links.append(OriginLink())
    for name, d, a, alpha, theta_offset, bounds in dh_Arm:
        links.append(
            DHLink(
                name=name,
                d=d,
                a=a,
                alpha=alpha,
                theta=theta_offset,  # offset en theta 
                bounds=bounds
            )
        )
    return Chain(name='arm_chain', links=links)

chain = build_arm_chain_from_dh()

# -----------------------------------------------
# 2) Semilla inicial para IK (longitud = nº de eslabones, aquí 5)
# -----------------------------------------------
ik_seed = [
    np.deg2rad(0),  # articulación 1
    np.deg2rad(0),  # articulación 2
    np.deg2rad(0),  # articulación 3
    np.deg2rad(0),  # articulación 4
    np.deg2rad(0),  # articulación 5
]

# -----------------------------------------------
# 3) Definir la orientación fija en RPY 
#    (roll=-90°, pitch=0°, yaw=+90°)
# -----------------------------------------------
target_orientation = geom.rpy_matrix(
    -np.pi/2,  # roll: -90°
     0.0,      # pitch: 0°
     np.pi/2   # yaw: +90°
)

# -----------------------------------------------
# 4) Definir la posición objetivo
# -----------------------------------------------
target_xyz = [0.3469, -0.010, 0.388]

# -----------------------------------------------
# 5) Calcular IK
# -----------------------------------------------
sol_full = chain.inverse_kinematics(
    target_position=target_xyz,
    target_orientation=target_orientation,
    max_iter=2500,
    regularization_parameter=0.001,
    optimizer='scalar',
    initial_position=ik_seed
)

print("Solución completa (radianes):", np.round(sol_full, 4))
print("Solución completa (grados):  ", np.round(np.rad2deg(sol_full), 2))

# -----------------------------------------------
# 6) Graficar la cadena en la solución de IK
# -----------------------------------------------
plt.ion()

# 6.1) Creamos la figura y el Axes3D
fig2 = plt.figure(figsize=(6, 6))
ax2  = fig2.add_subplot(111, projection='3d')  # <<< Aquí se crea el Axes3D, no un Figure

# 6.2) Dibujamos la cadena en la postura resultante de IK
chain.plot(ik_seed, ax2)  

# 6.3) Opcional: marcamos también el origen y el punto objetivo
ax2.scatter(0, 0, 0, color='red',   s=50, label='Base (0,0,0)')
ax2.scatter(
    target_xyz[0], target_xyz[1], target_xyz[2],
    color='blue', s=50, label='Objetivo'
)
ax2.set_title("Robot en solución de IK (IKPy + Matplotlib)")
ax2.legend()
ax2.set_proj_type('ortho')
ax2.set_xlim(0, 0.5)
ax2.set_ylim(0, 0.5)
ax2.set_zlim(0, 0.5)
plt.show()

plt.ion()

# 6.1) Creamos la figura y el Axes3D
fig = plt.figure(figsize=(6, 6))
ax  = fig.add_subplot(111, projection='3d')  # <<< Aquí se crea el Axes3D, no un Figure

# 6.2) Dibujamos la cadena en la postura resultante de IK
chain.plot(sol_full, ax)  

# 6.3) Opcional: marcamos también el origen y el punto objetivo
ax.scatter(0, 0, 0, color='red',   s=50, label='Base (0,0,0)')
ax.scatter(
    target_xyz[0], target_xyz[1], target_xyz[2],
    color='blue', s=50, label='Objetivo'
)
ax.set_title("Robot en solución de IK (IKPy + Matplotlib)")
ax.legend()
ax.set_proj_type('ortho')
ax.set_xlim(0, 0.5)
ax.set_ylim(0, 0.5)
ax.set_zlim(0, 0.5)
plt.show()
