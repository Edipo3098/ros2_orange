#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException
from ikpy.chain import Chain
from ikpy.link import DHLink
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
from tf_transformations import quaternion_from_matrix

# DH PARAMETERS (classic, ajustados según tu mensaje):
# DH:      d (m),      theta (rad), r (a) (m), alpha (rad)
dh_Arm = [
    #   name,           d,      a (r),      alpha,                theta_offset,          bounds
    ("articulacion1",  0.00,  0.0000, np.deg2rad(-180), np.deg2rad(90.0),   (np.deg2rad(-180), np.deg2rad(180))),
    ("articulacion2",  -0.1400,  0.000, np.deg2rad(90),   np.deg2rad(90),  (np.deg2rad(-180), np.deg2rad(180))),
    ("articulacion3",  0.0,  0.300, np.deg2rad(0), np.deg2rad(-90),    (np.deg2rad(-180), np.deg2rad(180))),
    ("articulacion4",  0.0,  0.14, np.deg2rad(-90),  np.deg2rad(0),  (np.deg2rad(-180), np.deg2rad(180))),
    ("articulacion5",  0.3800,  0, np.deg2rad(-90),   np.deg2rad(0.0),    (np.deg2rad(-180), np.deg2rad(180))),
]

# Offsets DH (en radianes), uno por articulación según tu tabla dh_Arm
theta_offsets = [
    np.deg2rad( 90.0),   # articulacion1  (antes 90°)
    np.deg2rad( 90.0),   # articulacion2  (antes 90°)
    np.deg2rad(-90.0),   # articulacion3  (antes -90°)
    np.deg2rad(  0.0),   # articulacion4  (antes   0°)
    np.deg2rad(  0.0),   # articulacion5  (antes   0°)
]

def build_arm_chain_from_dh():
    links = []
    for name, d, a, alpha, theta_offset, bounds in dh_Arm:
        links.append(
            DHLink(
                name=name,
                d=d,
                a=a,
                alpha=alpha,
                theta=theta_offset,  # ¡El offset lo sumamos en la solución!
                bounds=bounds
            )
        )
    return Chain(name='arm_chain', links=links)

def wrap_angle(angle):
    """Normaliza el ángulo a [-pi, pi]"""
    return (angle + np.pi) % (2 * np.pi) - np.pi

def saturate(val, lower, upper):
    return max(min(val, upper), lower)

class IKPyIKNode(Node):
    def __init__(self):
        super().__init__('ikpy_ik_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/arm_trajectory', 10)

        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tag_id', 5)
        self.declare_parameter('frequency', 10.0)
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.tag_id = self.get_parameter('tag_id').get_parameter_value().integer_value
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value

        self.chain = build_arm_chain_from_dh()
        self.get_logger().info(f"Cadena DH cargada: {[l.name for l in self.chain.links]}")
        self.ik_active = False
        self.create_timer(1.0 / self.frequency, self.timer_callback)

        # FK del home
        ee_home = self.chain.forward_kinematics([0, 0, 0, 0, 0])
        xh, yh, zh = ee_home[:3, 3]
        self.get_logger().info(f"[FK Home] EE posición DH home: ({xh:.3f}, {yh:.3f}, {zh:.3f})")
        # 1) Crea el broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # 2) Define los nombres de frames según tu DH
        self.joint_frames = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']

        # 3) Publica el home TF una sola vez si quieres
        ee_home = self.chain.forward_kinematics([0, 0, 0, 0, 0])
        self.publish_dh_frames([0, 0, 0, 0, 0])
        self.ik_seed = [
            
            1.6,    # articulacion1 ≈ 91.6°
            2.0,    # articulacion2 ≈114.6°
            1.5,    # articulacion3 ≈ 28.6°
            2.1,    # articulacion4 ≈ 0°
            0.0     # articulacion5 ≈ 0°
        ]

    def timer_callback(self):
        # Obtener TF al tag
        try:
            target_tag = f'tag36h11:{self.tag_id}'
            trans = self.tf_buffer.lookup_transform(
                self.base_frame, target_tag, rclpy.time.Time(), timeout=Duration(seconds=0.2))
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn(f"No se pudo obtener la pose de tag {self.tag_id}")
            self.publish_dh_frames([0, 0, 0, 0, 0])
            return

        tx, ty, tz = trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z
        self.get_logger().info(f"Tag {self.tag_id} en base: ({tx:.3f}, {ty:.3f}, {tz:.3f})")

        # TF del EE real para comparar
        try:
            trans_ee = self.tf_buffer.lookup_transform(
                self.base_frame, 'endEfector', rclpy.time.Time(), timeout=Duration(seconds=0.2))
            txEE, tyEE, tzEE = trans_ee.transform.translation.x, trans_ee.transform.translation.y, trans_ee.transform.translation.z

            self.get_logger().info(f"[TF Real] EE posición: ({txEE:.3f}, {tyEE:.3f}, {tzEE:.3f})")
            ee_home = self.chain.forward_kinematics([0, 0, 0, 0, 0])
            xh, yh, zh = ee_home[:3, 3]
            self.get_logger().info(f"[FK Home] EE posición DH home: ({xh:.3f}, {yh:.3f}, {zh:.3f})")
            
            

        except (LookupException, ConnectivityException, ExtrapolationException):
            txEE = tyEE = tzEE = float('nan')
            self.get_logger().warn("No se pudo obtener la pose del endEffector o joint5")

        try:
            # 1) Calcula la solución de IK con toda la cadena (5 articulaciones)
            #    Nota: IKPy devuelve un vector de longitud 6 = 1 "dummy" + 5 joints.
            sol_full = self.chain.inverse_kinematics(
                target_position=[tx, ty, tz],
                initial_position=self.ik_seed
            )
            # 2) Extrae las 5 posiciones DH
            joint_angles_dh = sol_full  # solu[0] es el dummy link base

            # 3) Convierte a ángulos físicos sumando offsets
            joint_angles_robot = []
            for idx, angle_dh in enumerate(joint_angles_dh):
                theta = wrap_angle(angle_dh )
                joint_angles_robot.append(theta)
                self.get_logger().info(
                    f"Art {idx+1}: θ_DH={angle_dh:.3f} rad ({np.rad2deg(angle_dh):.1f}°) | "
                    f"θ_robot={theta:.3f} rad ({np.rad2deg(theta):.1f}°)"
                )

            # 4) Comprueba el FK de esa solución DH
            fk_ik = self.chain.forward_kinematics(sol_full)
            x_fk, y_fk, z_fk = fk_ik[0,3], fk_ik[1,3], fk_ik[2,3]
            self.get_logger().info(f"[IKP ] EE DH: ({x_fk:.3f}, {y_fk:.3f}, {z_fk:.3f})")

            # 5) Publica la trayectoria con los 5 ángulos de tu robot
            traj = JointTrajectory()
            traj.joint_names = [l[0] for l in dh_Arm]
            point = JointTrajectoryPoint()
            point.positions = joint_angles_robot
            point.time_from_start = Duration(seconds=1.0).to_msg()
            traj.points.append(point)
            self.trajectory_pub.publish(traj)
            #self.get_logger().info(f"Trayectoria publicada: {[round(a,3) for a in joint_angles_robot]}")
            joint_angles_dh = sol_full
            self.publish_dh_frames(joint_angles_dh)

        except Exception as e:
            self.get_logger().error(f"Error al calcular IK/FK: {e}")
            

    def publish_dh_frames(self, angles):
        """
        Publica en /tf la cadena DH con los ángulos dados,
        esta vez calculando la traslación y orientación
        relativas eslabón→padre.
        """
        for i in range(len(self.chain.links)):
            # 1) pose global del eslabón i
            subchain_i = Chain(name=f'sub_i{i}', links=self.chain.links[:i+1])
            T_child = subchain_i.forward_kinematics(angles[:i+1])

            # 2) pose global del padre (o identidad si es base_link)
            if i == 0:
                T_parent = np.eye(4)
                parent_frame = 'base_link'
            else:
                subchain_p = Chain(name=f'sub_p{i}', links=self.chain.links[:i])
                T_parent = subchain_p.forward_kinematics(angles[:i])
                parent_frame = self.joint_frames[i-1]

            child_frame = self.joint_frames[i]

            # 3) transform relativo: T_rel = inverse(T_parent) * T_child
            T_rel = np.linalg.inv(T_parent) @ T_child

            # 4) extraer traslación
            x, y, z = T_rel[0,3], T_rel[1,3], T_rel[2,3]

            # 5) extraer quaternion de la matriz 4x4
            qx, qy, qz, qw = quaternion_from_matrix(T_rel)

            # 6) publicar
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id    = parent_frame
            tf.child_frame_id     = child_frame
            tf.transform.translation.x = float(x)
            tf.transform.translation.y = float(y)
            tf.transform.translation.z = float(z)
            tf.transform.rotation.x    = float(qx)
            tf.transform.rotation.y    = float(qy)
            tf.transform.rotation.z    = float(qz)
            tf.transform.rotation.w    = float(qw)

            self.tf_broadcaster.sendTransform(tf)



def main(args=None):
    rclpy.init(args=args)
    node = IKPyIKNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
