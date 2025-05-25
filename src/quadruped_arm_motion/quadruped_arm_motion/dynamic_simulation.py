#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient

class TrajClient(Node):
    def __init__(self):
        super().__init__('traj_client')
        # Nombre exacto de tu controlador
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/trajectory_controller/follow_joint_trajectory'
        )

    def send_goal(self, joint_names, positions, duration_s=2.0):
        # Esperamos a que el servidor de acción esté listo
        self._action_client.wait_for_server()

        # Creamos un mensaje JointTrajectory
        traj = JointTrajectory()
        traj.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration_s)
        point.time_from_start.nanosec = int((duration_s%1)*1e9)
        traj.points = [point]

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj

        # Enviamos el objetivo
        self._action_client.send_goal_async(goal_msg).add_done_callback(
            lambda future: self.get_logger().info("Goal enviado")
        )

def main(args=None):
    rclpy.init(args=args)
    client = TrajClient()

    # Lista de las 17 articulaciones, en el mismo orden que en tu YAML:
    joints = [
       'frontLeft_hip_motor_joint','frontLeft_knee_joint','frontLeft_ankle_joint',
       'frontRight_hip_motor_joint','frontRight_knee_joint','frontRight_ankle_joint',
       'backLeft_hip_motor_joint','backLeft_knee_joint','backLeft_ankle_joint',
       'backRight_hip_motor_joint','backRight_knee_joint','backRight_ankle_joint',
       'articulacion1','articulacion2','articulacion3','articulacion4','articulacion5'
    ]
    # Posiciones objetivo (en radianes) para cada joint
    target = [0.2, -0.5, 0.3,  0.2, -0.5, 0.3,   0.2, -0.5, 0.3,
              0.2, -0.5, 0.3,   0.0,  0.0, 0.0,   0.0, 0.0]

    client.send_goal(joints, target, duration_s=3.0)
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
