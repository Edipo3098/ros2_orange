#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException
from ikpy.chain import Chain
from ikpy.link import URDFLink
from ikpy.utils import geometry
import numpy as np

from ikpy.link import DHLink

def build_arm_chain_from_dh():
    # DH Parameters (ajusta si es necesario)
    dh_Arm = [
    # ("name",      d,         theta,          a (r),      alpha,           bounds)
    ("articulacion1", -0.0700, np.deg2rad(2.0),     0.0000, np.deg2rad(-89.0), (-np.pi, np.pi)),
    ("articulacion2",  0.0000, np.deg2rad(-174.4),  0.3035, np.deg2rad(0.0),   (-np.pi, np.pi)),
    ("articulacion3", -0.0001, np.deg2rad(83.6),    0.0724, np.deg2rad(-88.1), (-np.pi, np.pi)),
    ("articulacion4",  0.2827, np.deg2rad(-173.0),  0.0011, np.deg2rad(-87.9), (-np.pi, np.pi)),
    ("articulacion5",  0.0000, np.deg2rad(-0.0),    0.0500, np.deg2rad(0.0),   (-np.pi, np.pi)),
    ]

    links = []
    for name, d, a, alpha, theta_offset, bounds in dh_Arm:
        links.append(
            DHLink(
                name=name,
                d=d,
                a=a,
                alpha=alpha,
                theta=theta_offset,
                bounds=bounds
            )
        )

    return Chain(name='arm_chain', links=links)

class IKPyIKNode(Node):
    def __init__(self):
        super().__init__('ikpy_ik_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/arm_trajectory', 10)

        # --- PARÁMETROS ROS ---
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tag_id', 5)
        self.declare_parameter('frequency', 10.0)

        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.tag_id = self.get_parameter('tag_id').get_parameter_value().integer_value
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.frequency  = 1
        # --- Cadena DH ---
        self.chain = build_arm_chain_from_dh()
        self.get_logger().info(f"Cadena DH cargada: {[l.name for l in self.chain.links]}")

        self.ik_active = False
        self.create_timer(1.0 / self.frequency, self.timer_callback)

    def timer_callback(self):
        # 1. Obtener la pose de la base del brazo en 'map' (si existe)
        try:
            tf_base_map = self.tf_buffer.lookup_transform(
                'map', self.base_frame, rclpy.time.Time(), timeout=Duration(seconds=0.2))
            bx = tf_base_map.transform.translation.x
            by = tf_base_map.transform.translation.y
            bz = tf_base_map.transform.translation.z
            self.get_logger().info(f"Base real en map: ({bx:.3f}, {by:.3f}, {bz:.3f})")
        except Exception as e:
            self.get_logger().warn(f"No se pudo obtener la pose de base: {e}")
            bx = by = bz = 0.0

        # 2. Obtener el TF al AprilTag de interés desde la base del brazo
        try:
            target_tag = f'tag36h11:5'
            trans = self.tf_buffer.lookup_transform(
                self.base_frame, target_tag, rclpy.time.Time(), timeout=Duration(seconds=0.2))
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        tx = trans.transform.translation.x
        ty = trans.transform.translation.y
        tz = trans.transform.translation.z
        dist = math.sqrt(tx**2 + ty**2 + tz**2)

        #if dist < 0.5 and not self.ik_active:
        if True:
            self.get_logger().info(f"Tag 5 a {dist:.3f} m. Calculando IK...")
            # IKPy usa la pose deseada respecto a la base de la cadena
            # Si quieres compensar la base global, suma [bx, by, bz] a [tx, ty, tz] si lo ves necesario
            target_frame = self.chain.inverse_kinematics([tx, ty, tz])

            try:
                solution = self.chain.inverse_kinematics([tx, ty, tz])

                joint_angles = solution[1:]  # El primero suele ser fijo
                for i, angle in enumerate(joint_angles):
                    self.get_logger().info(f"Ángulo articulación {i+1}: {angle*(np.pi/180):.3f} rad")
                traj = JointTrajectory()
                
                traj.joint_names = traj.joint_names = ['articulacion1', 'articulacion2', 'articulacion3', 'articulacion4', 'articulacion5','ef']

                point = JointTrajectoryPoint()
                point.positions = joint_angles.tolist()

                point.time_from_start = Duration(seconds=1.0).to_msg()
                traj.points.append(point)

                self.trajectory_pub.publish(traj)
                self.get_logger().info(f"Trayectoria publicada: {joint_angles}")
                self.ik_active = True
            except Exception as e:
                self.get_logger().error(f"Error al calcular IK: {e}")
        elif dist >= 0.5 and self.ik_active:
            self.get_logger().info("Tag fuera de rango. Esperando reentrada...")
            self.ik_active = False

def main(args=None):
    rclpy.init(args=args)
    node = IKPyIKNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
