#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException
from std_msgs.msg import String,Bool
# RTB + SpatialMath
from roboticstoolbox.robot.DHLink import RevoluteDH
from roboticstoolbox.robot.DHRobot import DHRobot
from spatialmath import SE3

import numpy as np
import matplotlib.pyplot as plt


# Parámetros DH (5 DOF)
dh_Arm = [
    ("art1", 0.32,   0.00,   np.deg2rad(-180), np.deg2rad( 80.0), (np.deg2rad(-1.5),  np.deg2rad(1.5))),
    ("art2", -0.0800,0.00,   np.deg2rad(  90), np.deg2rad( 55.0), (np.deg2rad(-0.5),  np.deg2rad(30))),
    ("art3", 0.00,   0.30,   np.deg2rad(   0), np.deg2rad(-55.0), (np.deg2rad(-0.5),  np.deg2rad(40))),
    ("art4", 0.00,   0.1,   np.deg2rad(-90),  np.deg2rad(  30.0),(np.deg2rad(-40.5),  np.deg2rad(40))),
    ("art5", 0.3200, 0.00,   np.deg2rad(-90),  np.deg2rad(  0.0), (np.deg2rad(-0.5),  np.deg2rad(60))),
]



def build_arm_robot_from_dh():
    links = [
    RevoluteDH(d=d, a=a, alpha=alpha, offset=offset, qlim=bounds)
    for (_, d, a, alpha, offset, bounds) in dh_Arm
    ]
    robot = DHRobot(links, name='arm')
    return robot


class RTB_IK_Node(Node):
    def __init__(self):
        super().__init__('rtb_ik_node')

        # --- 1) TF listener y publicador de trayectorias ---
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/arm_trajectory', 10)
        self.subiscriber = self.create_subscription(Bool, '/calculate_Ik', self.trajectory_callback, 10)

        # --- 2) Parámetros ROS2 ---
        self.declare_parameter('base_frame', 'map')
        self.declare_parameter('tag_id', 5)
        self.declare_parameter('frequency', 5.0)  # 5 Hz de actualización
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.tag_id     = self.get_parameter('tag_id').get_parameter_value().integer_value
        self.frequency  = self.get_parameter('frequency').get_parameter_value().double_value

        # --- 3) Construir el DHRobot con RTB ---
        self.robot = build_arm_robot_from_dh()
        self.get_logger().info(f"Robot DH cargado con {len(self.robot.links)} eslabones")

        # --- 4) Semilla para IK y postura home ---
        self.qo = [10, 15, 0 ,26.57 ,10]
        self.home_angles = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.mask_translation = [0.3,0.3, 0.2, 0,0]
        self.target_xyz = [0.0, 0.0, 0.0]  # Inicializar posición objetivo
        self.q0 = np.zeros(5)  
        

        # --- 5) Inicializar Matplotlib en modo interactivo ---
        #plt.ion()
        # Dibujar inicialmente la postura home (en la figura #1)
        #self.plot_rtb(self.home_angles)

        # --- 6) Timer para llamar a timer_callback() periódicamente ---
        self.create_timer(1.0 / self.frequency, self.timer_callback)

    def trajectory_callback(self, msg):
        if msg.data:
            self.calculateIk()
    def timer_callback(self):
        # 1) Intentar obtener transform del AprilTag
        try:
            target_tag = f'tag36h11:{self.tag_id}'
            trans = self.tf_buffer.lookup_transform(
                self.base_frame, target_tag, rclpy.time.Time(), timeout=Duration(seconds=0.2)
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            #self.get_logger().warn(f"No se pudo obtener la pose de tag {self.tag_id}")
            return

        tx = trans.transform.translation.x
        ty = trans.transform.translation.y
        tz = trans.transform.translation.z
        self.get_logger().info(f"Tag {self.tag_id} en base: ({tx:.3f}, {ty:.3f}, {tz:.3f})")

        
        self.target_xyz = [tx, ty, tz]  # Solo la parte de traslación
             # rotation = identity, orientation libre

        
    def calculateIk(self):
        R_obj = SE3.RPY([0, 0, np.pi/2], order='xyz')
        self.target_xyz[2] = self.target_xyz[2] # Ajustar z para evitar colisiones
        self.get_logger().info(f"Tag {self.tag_id} en base: ({self.target_xyz[0]:.3f}, {self.target_xyz[1]:.3f}, {self.target_xyz[2]:.3f})")
        T_target     = SE3(*self.target_xyz) + R_obj      # rotation = identity, orientation libre
        # ------------------------------------------------------
        # 6) Intentar IK con Levenberg–Marquardt (solo translación)
        # ------------------------------------------------------
        
        q_lm, success_lm, iters_lm, searches_lm, residual_lm = self.robot.ik_LM(
            T_target,
            q0=self.q0,
            ilimit=2000,
            slimit=20000,
            tol=1e-4,
            mask=self.mask_translation,
            joint_limits=True
        )

        print(
            f"--- ik_LM --- éxito={success_lm}, "
            f"iter={iters_lm}, búsquedas={searches_lm}, residuo={residual_lm:.2e}"
        )
        print("Ángulos ik_LM (deg):", np.round(np.rad2deg(q_lm), 2))

        if success_lm:
            q_sol = q_lm
            metodo = "LM"
        else:
            q_sol = self.home_angles
            metodo = "Home"

        

        print(f"\nUsando la solución de {metodo} para animar…")
        if q_sol[1] > 0.45:
            q_sol[1] = 0.33  # Limitar el primer ángulo a 30 grados
            
        q_sol[1] = q_sol[1]-0.05  # Ajustar el primer ángulo para evitar colisiones
        q_solution = q_sol[1:]
        q_solution =np.append(q_solution, 0.5)  # Asegurar que el primer ángulo es positivo
        q_solution = np.abs(q_solution)  # Asegurar que todos los ángulos son positivos
        q_solution[4] = 0.110
        # 5) Publicar la trayectoria (opcional)
        traj = JointTrajectory()
        traj.joint_names = [l[0] for l in dh_Arm]
        pt = JointTrajectoryPoint()
        pt.positions =[float( val) for val in q_solution]
        pt.time_from_start = Duration(seconds=1.0).to_msg()
        traj.points.append(pt)
        
        self.trajectory_pub.publish(traj)

        # 6) **Actualizar el plot de Matplotlib** con la solución elegida
        #self.plot_rtb(q_sol)

        # 7) Extraer y loguear posición del efector final (opcional)
        T_ee = self.robot.fkine(q_solution)
        
        x_ee, y_ee, z_ee = T_ee.t.tolist()
        self.get_logger().info(f"Publicando trayectoria con ángulos: {np.round(np.rad2deg(q_solution), 2)}")
        
        self.get_logger().info(f"Pos. EFECTOR (RTB‐DH) → x={x_ee:.3f}, y={y_ee:.3f}, z={z_ee:.3f}")


    def plot_rtb(self, q):
        """
        Dibuja la configuración 'q' en Matplotlib usando RTB.  
        Para evitar que RTB cree “Figure 2” y que quede también una “Figure 1” en blanco, 
        hacemos lo siguiente:
        
        1) Cerramos/limpiamos todas las figuras previas con plt.close('all').
        2) Le decimos a RTB que use siempre la figura #1 llamando a fignum=1.
        3) Llamamos a plt.pause() para forzar el refresh.
        """
        # 1) Cerrar todas las figuras para que no queden ventanas antiguas
        plt.close('all')
        fig = plt.figure(1)

        # 2) Dibujar el robot en la figura número 1
        #    Con fignum=1 indica a RTB que pinte forzosamente en Figure 1.
        self.robot.plot(q, backend='pyplot', block=False)

        # 3) Forzar a Matplotlib a procesar eventos y actualizar la ventana
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    node = RTB_IK_Node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
