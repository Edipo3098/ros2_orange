#!/usr/bin/env python3
import rclpy
from rclpy.duration import Duration
from rclpy.node     import Node
from std_msgs.msg   import Float64MultiArray
from controller_manager_msgs.srv import SwitchController
from control_msgs.msg import DynamicJointState
POS_CTRL   = 'joint_group_position_controller'
EFF_CTRL   = 'joint_group_effort_controller'

class DynamicSimulationNode(Node):
    def __init__(self):
        super().__init__('dynamic_simulation')
        # Subscriber a joint_states
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
        # aquí guardamos el último effort leído
        self.latest_efforts = [0.0] * len(self.joint_names)
        self.sub_js = self.create_subscription(
            DynamicJointState,
            '/dynamic_joint_states',
            self.dynamic_state_cb,
            10)
        # ────────────────── publishers ──────────────────
        self.pub_pos = self.create_publisher(
            Float64MultiArray,
            f'/{POS_CTRL}/commands', 10)

        self.pub_eff = self.create_publisher(
            Float64MultiArray,
            f'/{EFF_CTRL}/commands', 10)
        
        # ────────────────── cliente al servicio ──────────────────
        self.cli_sw = self.create_client(
            SwitchController,
            '/controller_manager/switch_controller')

        # timers
        #self.create_timer(2.2,  self.send_pose_once)   # t = 1 s
        #self.create_timer(3.0,  self.do_switch_once)   # t = 4 s
        self.create_timer(2.2,  self.publish_efforts)  # t = 4.2 s +

        self.pose_sent = True
        self.switch_done = True
        self.done = False
    
    
    
    
    def dynamic_state_cb(self, msg: DynamicJointState):
        # msg.joint_names tiene N nombres; msg.interface_values[N] sus valores
        for j, joint_name in enumerate(msg.joint_names):
            if joint_name in self.joint_names:
                # posición en tu lista global
                idx = self.joint_names.index(joint_name)
                iv = msg.interface_values[j]
                # busca dónde está "effort"
                try:
                    k = iv.interface_names.index('effort')
                    self.latest_efforts[idx] = iv.values[k]*2
                except ValueError:
                    # si no hay 'effort' en esa interface, pon 0
                    self.latest_efforts[idx] = 0.0
    

    #  conmutar controladores -----------------------------------------------------
    def do_switch_once(self):
        if self.switch_done or not self.cli_sw.wait_for_service(timeout_sec=0.0):
            return                                              # espera al servicio

        req = SwitchController.Request()
        # campos NUEVOS (Iron-≥) ───────────────────────────────────────────────
        req.activate_controllers   = [EFF_CTRL]
        req.deactivate_controllers = [POS_CTRL]
        # estrictos: si algo falla, aborta
        req.strictness = SwitchController.Request.STRICT
        # timeout de 5 s
        req.timeout = Duration(seconds=1.0).to_msg()

        future = self.cli_sw.call_async(req)
        future.add_done_callback(self._confirm_switch)

    def _confirm_switch(self, future):
        try:
            if future.result().ok:
                self.get_logger().info('✓ Cambio a effort controller realizado.')
                self.switch_done = True
            else:
                self.get_logger().error('El cambio de controlador devolvió "ok = false".')
        except Exception as e:
            self.get_logger().error(f'Error al conmutar: {e}')

    # 3️  publicar torques -----------------------------------------------------------
    def publish_efforts(self):
        if self.done:
            return
        self.done = True
        τ_legs = 1.5    # torque para las 12 articulaciones de las piernas
        τ_arm  = 50.8    # torque para las 5 articulaciones del brazo

        # Construimos una lista de 12 veces τ_legs seguida de 5 veces τ_arm
        data_effor = [  0.2 ,0 , 0.0 ,  
                        0.2 ,0 , 0.0 ,
                        0.2 ,0 , 0.0 ,
                        0.2 ,0 , 0.0 ]
        
        msg_POS = Float64MultiArray()
        msg_POS.data = [0.0,  0.0, 0.0, 0.0, 0.0 ]
        self.pub_pos.publish(msg_POS)
        
        
        msg = Float64MultiArray(data=data_effor)
        self.pub_eff.publish(msg)
        self.get_logger().info('✓ Cambio a effort controller realizado.')
        # opcional: comprueba cuántos esfuerzos leíste
        self.get_logger().info(f"Latest efforts: {self.latest_efforts}")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DynamicSimulationNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
