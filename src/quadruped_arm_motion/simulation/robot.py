import numpy as np
import matplotlib.pyplot as plt

from roboticstoolbox.robot.DHLink import RevoluteDH
from roboticstoolbox.robot.DHRobot import DHRobot
from spatialmath import SE3
import roboticstoolbox as rtb  # solo para jtraj
puma = rtb.models.DH.Puma560() 
base = [0 , 0, 0.3]  # posición objetivo en el espacio (x, y, z)
T_base     = SE3(*base) 
# ------------------------------------------------------
# 1) Construir el robot DH de 5 eslabones
# ------------------------------------------------------
dh_Arm = [
    ("art1", 0.32,   0.00,   np.deg2rad(-180), np.deg2rad( 80.0), (np.deg2rad(-1.5),  np.deg2rad(1.5))),
    ("art2", -0.0800,0.00,   np.deg2rad(  90), np.deg2rad( 55.0), (np.deg2rad(-0.5),  np.deg2rad(65))),
    ("art3", 0.00,   0.30,   np.deg2rad(   0), np.deg2rad(-45.0), (np.deg2rad(-0.5),  np.deg2rad(40))),
    ("art4", 0.00,   0.1,   np.deg2rad(-90),  np.deg2rad(  30.0),(np.deg2rad(-40.5),  np.deg2rad(40))),
    ("art5", 0.3200, 0.00,   np.deg2rad(-90),  np.deg2rad(  0.0), (np.deg2rad(-0.5),  np.deg2rad(180))),
]
links = [
    RevoluteDH(d=d, a=a, alpha=alpha, offset=offset, qlim=bounds)
    for (_, d, a, alpha, offset, bounds) in dh_Arm
]
robot = DHRobot(links, name='arm')
# ------------------------------------------------------
# 2) Postura home (todas las articulaciones en 0)
# ------------------------------------------------------
home = [0.0, 0.0, 0.0, 0.0,0.0]


# ------------------------------------------------------
# 3) Definir objetivo puramente posicional
#    (sin rotación, es decir “orientación libre”)
# ------------------------------------------------------
target_xyz_real = [0.378, -0.012, 0.323]    # cualquier punto dentro del workspace
target_xyz = target_xyz_real
target_xyz = [0.378, -0.112, 0.323]     # cualquier punto dentro del workspace
R_obj = SE3.RPY([0, 0, -np.pi/2], order='xyz')
T_target     = SE3(*target_xyz_real)* R_obj      # rotation = identity, orientation libre



# 3) Definir la postura 'home' (todas las articulaciones en 0)
# ------------------------------------------------------

# 4) Semilla inicial para los solvers de IK
# ------------------------------------------------------
q0 = np.zeros(5)
if target_xyz[1] < 0.0:
    q0[0] = np.deg2rad(np.deg2rad(0))  # Articulación 1: rotación de 180° si Y negativo
    q0[1] = np.deg2rad(np.deg2rad(0))  # Articulación 1: rotación de 180° si Y negativo
# ------------------------------------------------------
# 5) Máscara para ignorar rotación (solo se resuelve posición)
# ------------------------------------------------------
mask_translation = [0.3,0.3, 0.2, 0,0]

# ------------------------------------------------------
# 10) FIGURA 1 → mostrar el robot en home
# ------------------------------------------------------
plt.ion()
fig_home = plt.figure(1)
plt.clf()                                 # Limpiar ventana 1
robot.plot(home, backend='pyplot', fig=fig_home, block=False)
ax = fig_home.gca()
ax2 = fig_home.gca()
ax.scatter([0], [0], [0], color='red', s=50, label='Origen (0,0,0)')
ax2.scatter(target_xyz[0], target_xyz[1], target_xyz[2], color='blue', s=50, label='Origen (0,0,0)')
plt.title("Figura 1: Robot en Home")
plt.pause(1.0)                            # Mantener 1s para que se vea el home claramente

# ------------------------------------------------------
# 6) Intentar IK con Levenberg–Marquardt (solo translación)
# ------------------------------------------------------
q_lm, success_lm, iters_lm, searches_lm, residual_lm = robot.ik_LM(
    T_target,
    start = "art1",
    q0=q0,
    ilimit=1000,
    slimit=20000,
    tol=1e-4,
    mask=mask_translation,
    joint_limits=True,
   
)

print(
    f"--- ik_LM --- éxito={success_lm}, "
    f"iter={iters_lm}, búsquedas={searches_lm}, residuo={residual_lm:.2e}"
)
print("Ángulos ik_LM (deg):", np.round(np.rad2deg(q_lm), 2) ," (rad):",(0.00, 2))


# ------------------------------------------------------
# 7) Si LM falla, probar Gauss–Newton
# ------------------------------------------------------

#q_gn, success_gn, iters_gn, searches_gn, residual_gn = robot.ik_GN(
#    T_target,
#    q0=q0,
#    start = "art1",
#    ilimit=2000,
#    slimit=200,
#    tol=1e-4,
#    mask=mask_translation,
#    joint_limits=True,
#    pinv=False
#)
#print(
#    f"--- ik_GN --- éxito={success_gn}, "
#    f"iter={iters_gn}, búsquedas={searches_gn}, residuo={residual_gn:.2e}"
#)
#print("Ángulos ik_GN (deg):", np.round(np.rad2deg(q_gn), 2)," (rad):",(0.00, 2))
## ------------------------------------------------------
## 8) Si GN falla, probar Newton–Raphson
## ------------------------------------------------------
#q_nr, success_nr, iters_nr, searches_nr, residual_nr = robot.ik_NR(
#    T_target,
#    q0=q0,
#    start = "art1",
#    ilimit=2000,
#    slimit=200,
#    tol=1e-6,
#    mask=mask_translation,
#    joint_limits=True,
#    pinv=False
#)
#print(
#    f"--- ik_NR --- éxito={success_nr}, "
#    f"iter={iters_nr}, búsquedas={searches_nr}, residuo={residual_nr:.2e}"
#)
#
#print("Ángulos ik_NR (deg):", np.round(np.rad2deg(q_nr), 2)," (rad):",(0.00, 2))
if success_lm:
    q_sol = q_lm
    metodo = "LM"
#elif success_gn:
#    q_sol = q_gn
#    metodo = "GN"
#elif success_nr:
#    q_sol = q_nr
#    metodo = "NR"
else:
    print("Ningún método de IK funcionó.")
    exit (1)
    q_sol = None
    metodo = "Ninguno"
 
q_sol = q_lm
metodo = "LM"

print(f"\nUsando la solución de {metodo} para animar…")
q_sol[0]  = 0
# ------------------------------------------------------
# 9) Generar una trayectoria en espacio articular (50 pasos)
# ------------------------------------------------------
traj   = rtb.jtraj(home, np.rad2deg(q_sol), 50)  # RTB espera grados
q_traj = np.deg2rad(traj.q)                     # volvemos a radianes


#
## ------------------------------------------------------
## 11) FIGURA 2 → animación Home → Solución IK
## ------------------------------------------------------
fig_anim = plt.figure(2)

for qk in q_traj:
    plt.clf()                             # Limpiar solo la Figura 2
    robot.plot(qk, backend='pyplot', fig=fig_anim, block=False)
    ax3 = fig_anim.gca()
    ax4 = fig_anim.gca()
    ax3.scatter([0], [0], [0], color='red', s=50, label='Origen (0,0,0)')
    ax4.scatter(target_xyz[0], target_xyz[1], target_xyz[2], color='blue', s=50, label='Origen (0,0,0)')
    plt.title("Figura 2: Trayectoria Home → Solución IK")
    plt.pause(0.01)                       # 50 ms entre frames

# ------------------------------------------------------
# 12) Mantener ambas ventanas abiertas hasta que el usuario cierre
# ------------------------------------------------------
plt.ioff()
plt.show()
#