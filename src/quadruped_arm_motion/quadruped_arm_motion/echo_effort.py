#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

EFF_CTRL = 'joint_group_effort_controller'

class EchoEffortNode(Node):
    def __init__(self):
        super().__init__('echo_effort')
        # publisher al controller de esfuerzo
        self.pub_eff = self.create_publisher(
            Float64MultiArray,
            f'/{EFF_CTRL}/commands',
            10)

        # suscripción a JointState
        self.joint_names = [
          'frontLeft_hip_motor_joint',
          'frontLeft_knee_joint',
          'frontLeft_ankle_joint',
          'frontRight_hip_motor_joint',
          'frontRight_knee_joint',
          'frontRight_ankle_joint',
          'backLeft_hip_motor_joint',
          'backLeft_knee_joint',
          'backLeft_ankle_joint',
          'backRight_hip_motor_joint',
          'backRight_knee_joint',
          'backRight_ankle_joint',
          'articulacion1',
          'articulacion2',
          'articulacion3',
          'articulacion4',
          'articulacion5',
        ]
        self.sub_js = self.create_subscription(
            JointState,
            '/dynamic_joint_states ',
            self.joint_state_cb,
            10)

    def joint_state_cb(self, msg: JointState):
        # msg.name es la lista de todos los joints, msg.effort el torque asociado
        # extraemos sólo los que nos interesan, en el mismo orden
        efforts = []
        for name in self.joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                efforts.append(msg.effort[idx])
            else:
                efforts.append(0.0)  # por si faltara alguno
        # y lo republicamos
        cmd = Float64MultiArray(data=efforts)
        self.pub_eff.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = EchoEffortNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
