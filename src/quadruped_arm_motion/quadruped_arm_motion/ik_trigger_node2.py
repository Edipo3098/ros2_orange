#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException

# Importar RTB y SpatialMath
from roboticstoolbox.robot.DHLink import RevoluteDH      # Para definir eslabones DH :contentReference[oaicite:0]{index=0}
from roboticstoolbox.robot.DHRobot import DHRobot        # Para construir un DHRobot a partir de enlaces :contentReference[oaicite:1]{index=1}
from spatialmath import SE3                              # Para representar poses SE(3) :contentReference[oaicite:2]{index=2}

import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_matrix


# Parámetros DH (misma estructura que antes)
dh_Arm = [
    ("articulacion1",  0.00   ,  0.00 , np.deg2rad(-180), np.deg2rad(90.0),  (np.deg2rad(0),   np.deg2rad(180))),
    ("articulacion2", -0.1400 ,  0.00 , np.deg2rad(90) ,  np.deg2rad(90) ,     (np.deg2rad(0),   np.deg2rad(180))),
    ("articulacion3",  0.0    ,  0.30 , np.deg2rad(0.0),   np.deg2rad(-90),     (np.deg2rad(0),   np.deg2rad(180))),
    ("articulacion4",  0.0    ,  0.14 , np.deg2rad(-90),   np.deg2rad(0.0),      (np.deg2rad(0),   np.deg2rad(180))),
    ("articulacion5",  0.3800 ,  0.0  , np.deg2rad(-90),   np.deg2rad(0.0),      (np.deg2rad(0),   np.deg2rad(180))),
]

theta_offsets = [
    np.deg2rad( 90.0),
    np.deg2rad( 90.0),
    np.deg2rad(-90.0),
    np.deg2rad(  0.0),
    np.deg2rad(  0.0),
]

def build_arm_robot_from_dh():
    """
    Construye un objeto DHRobot a partir de los parámetros dh_Arm usando RevoluteDH.
    """
    links = []
    for name, d, a, alpha, theta_offset, bounds in dh_Arm:
        link = RevoluteDH(
            d=d,
            a=a,
            alpha=alpha,
            offset=theta_offset,
            qlim=bounds
        )  # :contentReference[oaicite:3]{index=3}
        links.append(link)
    return DHRobot(links, name='arm')  # :contentReference[oaicite:4]{index=4}


class RTB_IK_Node(Node):
    def __init__(self):
        super().__init__('rtb_ik_node')

        # Inicializar TF y publicador de trayectoria
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/arm_trajectory', 10)

        # Parámetros ROS2
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tag_id', 5)
        self.declare_parameter('frequency', 10.0)
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.tag_id = self.get_parameter('tag_id').get_parameter_value().integer_value
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value

        # Construir robot con RTB
        self.robot = build_arm_robot_from_dh()
        self.get_logger().info(f"Robot DH cargado: {len(self.robot.links)} eslabones")

        # Crear broadcaster de TF para publicar cada articulación
        self.tf_broadcaster = TransformBroadcaster(self)
        self.joint_frames = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']

        # Timer para llamar a timer_callback a la frecuencia deseada
        self.create_timer(1.0 / self.frequency, self.timer_callback)

        # Semilla inicial (5 DOF)
        self.ik_seed = [
            np.deg2rad(10),
            np.deg2rad(30),
            np.deg2rad(10),
            np.deg2rad(50),
            np.deg2rad(50),
        ]

    def timer_callback(self):
        # 1) Obtener TF del tag April
        try:
            target_tag = f'tag36h11:{self.tag_id}'
            trans = self.tf_buffer.lookup_transform(
                self.base_frame, target_tag, rclpy.time.Time(), timeout=Duration(seconds=0.2)
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn(f"No se pudo obtener la pose de tag {self.tag_id}")
            self.publish_dh_frames([0, 0, 0, 0, 0])
            return

        tx = trans.transform.translation.x
        ty = trans.transform.translation.y
        tz = trans.transform.translation.z
        self.get_logger().info(f"Tag {self.tag_id} en base: ({tx:.3f}, {ty:.3f}, {tz:.3f})")

        # 2) Construir pose objetivo T_target = Trans(tx,ty,tz) * Rot(roll=-90°,pitch=0°,yaw=+90°)
        R_obj = SE3.RPY([-np.pi/2, 0.0, np.pi/2], order='xyz')  # :contentReference[oaicite:5]{index=5}
        T_target = SE3(tx, ty, tz) * R_obj                           # :contentReference[oaicite:6]{index=6}

        # 3) Convertir semilla a numpy.ndarray
        q0_np = np.asarray(self.ik_seed)

        # === Invocar los tres solvers C++ Rápidos ===
        # 3.1) Solver Levenberg–Marquardt (ik_LM)
        try:
            q_lm, success_lm, iters_lm, searches_lm, residual_lm = self.robot.ik_LM(
                T_target,
                q0=q0_np,
                ilimit=500,
                slimit=100,
                tol=1e-6,
            )  # :contentReference[oaicite:7]{index=7}
        except Exception as e:
            self.get_logger().error(f"ik_LM falló: {e}")
            return

        # 3.2) Solver Gauss–Newton (ik_GN)
        try:
            q_gn, success_gn, iters_gn, searches_gn, residual_gn = self.robot.ik_GN(
                T_target,
                q0=q0_np,
                ilimit=500,
                slimit=100,
                tol=1e-6,
            )  # :contentReference[oaicite:8]{index=8}
        except Exception as e:
            self.get_logger().error(f"ik_GN falló: {e}")
            return

        # 3.3) Solver Newton–Raphson (ik_NR)
        try:
            q_nr, success_nr, iters_nr, searches_nr, residual_nr = self.robot.ik_NR(
                T_target,
                q0=q0_np,
                ilimit=500,
                slimit=100,
                tol=1e-6,
            )  # :contentReference[oaicite:9]{index=9}
        except Exception as e:
            self.get_logger().error(f"ik_NR falló: {e}")
            return

        # 4) Mostrar resultados en el logger
        self.get_logger().info(f"--- Resultados ik_LM  --- success={success_lm}, iter={iters_lm}, searches={searches_lm}, residual={residual_lm:.2e}")
        self.get_logger().info(f"Ángulos ik_LM: {np.round(np.rad2deg(q_lm), 1)}°")

        self.get_logger().info(f"--- Resultados ik_GN  --- success={success_gn}, iter={iters_gn}, searches={searches_gn}, residual={residual_gn:.2e}")
        self.get_logger().info(f"Ángulos ik_GN: {np.round(np.rad2deg(q_gn), 1)}°")

        self.get_logger().info(f"--- Resultados ik_NR  --- success={success_nr}, iter={iters_nr}, searches={searches_nr}, residual={residual_nr:.2e}")
        self.get_logger().info(f"Ángulos ik_NR: {np.round(np.rad2deg(q_nr), 1)}°")

        # 5) Elegir cuál usar (aquí usaremos ik_LM para generar la trayectoria)
        if success_lm:
            joint_solution = q_lm.tolist()
        else:
            self.get_logger().warn("ik_LM no convergió, no se publica trayectoria")
            return

        # 6) Publicar trayectoria de 1 segundo hacia la solución elegida
        traj = JointTrajectory()
        traj.joint_names = [l[0] for l in dh_Arm]
        point = JointTrajectoryPoint()
        point.positions = joint_solution
        point.time_from_start = Duration(seconds=1.0).to_msg()
        traj.points.append(point)
        self.trajectory_pub.publish(traj)

        # 7) Publicar en TF la cadena DH con los ángulos finales (ik_LM)
        self.publish_dh_frames(joint_solution)

    def publish_dh_frames(self, angles):
        """Publica en /tf la cadena DH con los ángulos dados."""
        n = len(self.robot.links)
        for i in range(n):
            # Transformación global del eslabón i
            T_child = self.robot.A(i + 1, angles)  # :contentReference[oaicite:10]{index=10}

            # Transformación global del padre (o identidad si i=0)
            if i == 0:
                T_parent = SE3()  # Identidad
                parent_frame = 'base_link'
            else:
                T_parent = self.robot.A(i, angles)  # :contentReference[oaicite:11]{index=11}
                parent_frame = self.joint_frames[i - 1]

            # Transformación relativa
            T_rel = T_parent.inv() * T_child  # :contentReference[oaicite:12]{index=12}

            # Extraer traslación
            x, y, z = T_rel.t

            # Extraer cuaternión (w, x, y, z)
            qx, qy, qz, qw = T_rel.unit_quaternion  # :contentReference[oaicite:13]{index=13}

            # Publicar TransformStamped
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = parent_frame
            tf.child_frame_id = self.joint_frames[i]
            tf.transform.translation.x = float(x)
            tf.transform.translation.y = float(y)
            tf.transform.translation.z = float(z)
            tf.transform.rotation.x = float(qx)
            tf.transform.rotation.y = float(qy)
            tf.transform.rotation.z = float(qz)
            tf.transform.rotation.w = float(qw)
            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = RTB_IK_Node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
