#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException
from std_msgs.msg import String,Bool
# RTB + SpatialMath

from ikpy.chain import Chain
from ikpy.link import DHLink
import ikpy.utils.geometry as geom

import numpy as np
import matplotlib.pyplot as plt

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
        self.robot = build_arm_chain_from_dh()
        self.get_logger().info(f"Robot DH cargado con {len(self.robot.links)} eslabones")

        # --- 4) Semilla para IK y postura home ---
        self.qo = [10, 15, 0 ,26.57 ,10]
        self.home_angles = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.mask_translation = [0.8, 0.5, 0.8, 0, 0,0.2]
        self.target_xyz = [0.0, 0.0, 0.0]  # Inicializar posición objetivo
        self.target_orientation = geom.rpy_matrix(-np.pi/2,  # roll: -90°
                                    0.0,      # pitch: 0°
                                    np.pi/2   # yaw: +90°
                                )
        self.ik_seed = [
            np.deg2rad(0),  # articulación 1
            np.deg2rad(0),  # articulación 2
            np.deg2rad(0),  # articulación 3
            np.deg2rad(0),  # articulación 4
            np.deg2rad(0),  # articulación 5
        ]

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
        self.get_logger().info(f"Tag {self.tag_id} en base: ({self.target_xyz[0]:.3f}, {self.target_xyz[1]:.3f}, {self.target_xyz[2]:.3f})")
           # rotation = identity, orientation libre
        # ------------------------------------------------------
        # 6) Intentar IK con Levenberg–Marquardt (solo translación)
        # ------------------------------------------------------
       
        sol_full = self.robot.inverse_kinematics(
        target_position= self.target_xyz,
        target_orientation=self.target_orientation,
        max_iter=2500,
        regularization_parameter=0.001,
        optimizer='scalar',
        initial_position=self.ik_seed
    )
        
        sol_full[0] = np.abs(sol_full[0])  # Asegurar que el primer ángulo es positivo
        sol_full[1] = np.abs(sol_full[1])  # Asegurar que el segundo ángulo es positivo
        sol_full[2] = np.abs(sol_full[2])  # Asegurar que el tercer ángulo es positivo
        sol_full[3] = np.abs(sol_full[3])  # Asegurar que el cuarto ángulo es positivo
        sol_full[4] = np.abs(sol_full[4])  # Asegurar que el quinto ángulo es positivo
        self.get_logger().info(f"Solución completa (radianes): {np.round(sol_full, 1)}")
        self.get_logger().info(f"Solución completa (grados):  {np.round(np.rad2deg(sol_full), 1)}")
        if sol_full[0] > 25:
            sol_full[0] = 15 
        if sol_full[2] < 0:
            sol_full[2] = 0  
        # 5) Publicar la trayectoria (opcional)
        traj = JointTrajectory()
        traj.joint_names = [l[0] for l in dh_Arm]
        pt = JointTrajectoryPoint()
        pt.positions =[float( val) for val in sol_full]
        pt.time_from_start = Duration(seconds=1.0).to_msg()
        traj.points.append(pt)
        
        self.trajectory_pub.publish(traj)


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
