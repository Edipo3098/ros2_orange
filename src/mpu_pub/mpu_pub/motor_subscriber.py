import rclpy
from rclpy.node import Node
from robot_interfaces.msg import MoveRobot, Command
import time
import serial

START_DELIM = b'<'
END_DELIM   = b'>'
READ_TIMEOUT = 1.0  # s

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('motor_subscriber')
        self.sub = self.create_subscription(
            MoveRobot, 'motor_angles', self.listener_callback, 10)
        self.pub = self.create_publisher(Command, 'command_robot', 10)

        self.msg_cmd = Command()
        self.reset_cmd_flags()
        self.msg_cmd.ready = True
        self.pub.publish(self.msg_cmd)

        self.Sending = False
        self.timer_comm = self.create_timer(2.0, self.check_arduino)
        self.get_logger().info('Motor subscriber ready')

    def reset_cmd_flags(self):
        self.msg_cmd.armmoving = False
        self.msg_cmd.grippermoving = False
        self.msg_cmd.gripperopen = False
        self.msg_cmd.gripperclosed = False
        self.msg_cmd.quadmoving = False

    def read_packet(self, ser: serial.Serial) -> str:
        """
        Lee una trama <...> y devuelve el payload sin delimitadores.
        Retorna '' si no recibió nada válido dentro de READ_TIMEOUT.
        """
        start = time.time()
        buf = b''
        # busca inicio
        while time.time() - start < READ_TIMEOUT:
            b = ser.read(1)
            if b == START_DELIM:
                break
        else:
            return ''
        # lee hasta fin
        while time.time() - start < READ_TIMEOUT:
            b = ser.read(1)
            if not b:
                continue
            if b == END_DELIM:
                break
            # sólo imprimibles
            if 32 <= b[0] <= 126:
                buf += b
        try:
            return buf.decode('ascii').strip()
        except UnicodeDecodeError:
            return buf.decode('ascii', errors='ignore').strip()

    def check_arduino(self):
        if self.Sending:
            return
        try:
            with serial.Serial('/dev/ttyS5', 115200, timeout=0.1) as ser:
                ser.reset_input_buffer()
                ser.write(b'<CHECK>\n')
                resp = self.read_packet(ser)
                if resp == 'CHECK':
                    self.get_logger().info('Arduino OK')
                    self.msg_cmd.ready = True
                else:
                    self.get_logger().warn(f'Arduino NOK (“{resp}”)')
                    self.msg_cmd.ready = False
                self.pub.publish(self.msg_cmd)
        except serial.SerialException as e:
            self.get_logger().error(f'Serial error: {e}')

    def listener_callback(self, msg):
        self.Sending = True
        self.timer_comm.cancel()

        cmd = msg.command
        self.get_logger().info(f'Got command "{cmd}"')

        # Ejemplo de ARM vs gait vs origin...
        payload = None
        if cmd == 'ARM':
            self.msg_cmd.armmoving = True
            payload = f'ARM,{msg.m0},{msg.m1},{msg.m2},{msg.m3},{msg.m4},{msg.m5}'
        elif cmd == 'm4':
            self.msg_cmd.quadmoving = True
            payload = 'm3'
        else:
            payload = 'origin'

        # envía y espera ACK
        try:
            with serial.Serial('/dev/ttyS5', 115200, timeout=0.1) as ser:
                ser.reset_input_buffer()
                ser.write(f'<{payload}>\n'.encode('ascii'))
                # espera respuesta TRUE/FALSE
                for _ in range(20):
                    resp = self.read_packet(ser)
                    if resp in ('TRUE','FALSE'):
                        break
                else:
                    resp = 'FALSE'

                ok = (resp == 'TRUE')
                if ok:
                    self.get_logger().info('Arduino responded TRUE')
                    self.msg_cmd.ready = True
                else:
                    self.get_logger().warn(f'Arduino responded: "{resp}"')
                    self.msg_cmd.ready = False

        except serial.SerialException as e:
            self.get_logger().error(f'Serial error: {e}')
            self.msg_cmd.ready = False

        # publica estado y reactiva timer
        self.pub.publish(self.msg_cmd)
        self.reset_cmd_flags()
        self.Sending = False
        self.timer_comm = self.create_timer(2.0, self.check_arduino)


def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
