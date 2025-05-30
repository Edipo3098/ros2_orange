#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException
from ikpy.chain import Chain
import numpy as np
import os

class IKPyURDFNode(Node):
    def __init__(self):
        super().__init__('ikpy_urdf_ik_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/arm_trajectory', 10)

        self.declare_parameter('base_frame', 'M0_big_fixed')
        self.declare_parameter('end_effector_frame', 'endEfector')
        self.declare_parameter('frequency', 1.0)
        self.declare_parameter('robot_urdf_path', '/home/edipo/ros2_orange/src/quadruped_arm_motion/urdf/quadruped_arm_robot_control_real_effortTest_develope.urdf')

        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.end_effector_frame = self.get_parameter('end_effector_frame').get_parameter_value().string_value
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.robot_urdf_path = self.get_parameter('robot_urdf_path').get_parameter_value().string_value

        if not os.path.exists(self.robot_urdf_path):
            self.get_logger().error(f"URDF file not found: {self.robot_urdf_path}")
            raise RuntimeError("URDF not found!")

        # Lista de joints activos del brazo
        self.joint_names = [
            "articulacion1",
            "articulacion2",
            "articulacion3",
            "articulacion4",
            "articulacion5"
        ]

        self.chain = Chain.from_urdf_file(self.robot_urdf_path)
        for idx, link in enumerate(self.chain.links):
            self.get_logger().info(f"{idx}: {link.name} | class: {link.__class__.__name__}")

        self.get_logger().info(f"IKPy URDF Chain: {[l.name for l in self.chain.links]}")

        self.create_timer(1.0 / self.frequency, self.timer_callback)

    def timer_callback(self):
        # Obtener la pose deseada desde tf2 (por ejemplo: al AprilTag)
        try:
            target_frame = 'tag36h11:5'
            trans = self.tf_buffer.lookup_transform(
                self.base_frame, target_frame, rclpy.time.Time(), timeout=Duration(seconds=0.2))
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"No TF para {target_frame}: {e}")
            return

        tx = trans.transform.translation.x
        ty = trans.transform.translation.y
        tz = trans.transform.translation.z
        self.get_logger().info(f"Target {target_frame} en {self.base_frame}: ({tx:.3f}, {ty:.3f}, {tz:.3f})")

        # Resuelve IKPy usando solo la posición, puedes también usar orientación si quieres
        target_xyz = [tx, ty, tz]
        try:
            ik_solution = self.chain.inverse_kinematics(target_xyz)
            joint_angles = ik_solution[1:6]  # Si solo tienes 5 joints activos
            self.get_logger().info(f"Ángulos calculados: {[f'{np.rad2deg(a):.2f}' for a in joint_angles]} (deg)")

            # Comprobar la posición alcanzada por la cadena
            ee_matrix = self.chain.forward_kinematics(ik_solution)
            ee_x, ee_y, ee_z = ee_matrix[:3, 3]
            self.get_logger().info(f"FK IKPy: ({ee_x:.3f}, {ee_y:.3f}, {ee_z:.3f})")

            traj = JointTrajectory()
            traj.joint_names = self.joint_names
            point = JointTrajectoryPoint()
            point.positions = joint_angles.tolist()
            point.time_from_start = Duration(seconds=1.0).to_msg()
            traj.points.append(point)
            self.trajectory_pub.publish(traj)
            self.get_logger().info("Trayectoria publicada.")

        except Exception as e:
            self.get_logger().error(f"Error en IKPy: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = IKPyURDFNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
