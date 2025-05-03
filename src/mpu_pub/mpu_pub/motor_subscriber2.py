import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Anglemotor, MoveRobot, Command
import asyncio
import serial_asyncio
import time

class SerialReader(asyncio.Protocol):
    def __init__(self, node):
        self.node = node
        self.buffer = ''

    def connection_made(self, transport):
        self.transport = transport
        self.node.get_logger().info('Serial port opened')
        self.node.serial_transport = transport

    def data_received(self, data):
        text = data.decode()
        self.buffer += text
        if '\n' in self.buffer:
            lines = self.buffer.split('\n')
            for line in lines[:-1]:
                self.node.handle_serial_input(line.strip())
            self.buffer = lines[-1]

    def connection_lost(self, exc):
        self.node.get_logger().warn('Serial port closed!')
        self.node.serial_transport = None

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('motor_subscriber2')
        self.subscription = self.create_subscription(MoveRobot, 'motor_angles', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Command, 'command_robot', 10)
        self.msg_command = Command()
        self.msg_command.ready = True
        self.msg_command.armmoving = False
        self.msg_command.grippermoving = False
        self.msg_command.gripperopen = False
        self.msg_command.gripperclosed = False
        self.msg_command.quadmoving = False
        self.publisher_.publish(self.msg_command)
        self.isARM = False
        self.isGait = False
        self.Sending = False
        self.tryng_coommunication = 0
        self.serial_transport = None
        self.get_logger().info('Publish true')

    def handle_serial_input(self, line):
        self.get_logger().info(f'Received from serial: "{line}"')
        if line == "check":
            self.get_logger().info('Communication with Arduino is OK')
            self.msg_command.ready = True
        elif line == "True":
            self.msg_command.ready = True
            self.get_logger().info('Movement finished')
        else:
            self.tryng_coommunication += 1
            if self.tryng_coommunication >= 150:
                self.get_logger().info('Communication with Arduino is NOT OK')
                self.msg_command.ready = False
            else:
                self.msg_command.ready = False
        self.msg_command.armmoving = False
        self.msg_command.grippermoving = False
        self.msg_command.gripperopen = False
        self.msg_command.gripperclosed = False
        self.msg_command.quadmoving = False
        self.publisher_.publish(self.msg_command)

    def listener_callback(self, msg):
        self.Sending = True
        self.get_logger().info('Command "%s"' % msg.command)

        if msg.command == "ARM":
            self.isARM = True
        elif msg.command == "m4":
            msg.command = "m3"
            self.isGait = True
        else:
            msg.command = "origin"
            self.isARM = False

        if self.serial_transport is None:
            self.get_logger().warn("Serial port not available!")
            return

        try:
            if self.isARM:
                self.serial_transport.write((msg.command + "\n").encode())
                time.sleep(0.2)
                csv_line = f"{msg.m0},{msg.m1},{msg.m2},{msg.m3},{msg.m4},{2}\n"
                self.serial_transport.write(csv_line.encode())
                self.isARM = False
                self.msg_command.armmoving = True
                self.msg_command.quadmoving = False
            elif self.isGait:
                self.serial_transport.write((msg.command + "\n").encode())
                self.isGait = False
                self.msg_command.armmoving = False
                self.msg_command.quadmoving = True
            else:
                self.serial_transport.write((msg.command + "\n").encode())
                self.isARM = False

            self.msg_command.ready = False
            self.publisher_.publish(self.msg_command)

        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")
        self.Sending = False


async def main_async():
    rclpy.init()
    node = MinimalSubscriber()
    loop = asyncio.get_running_loop()

    # Conexión serial usando serial_asyncio
    try:
        transport, protocol = await serial_asyncio.create_serial_connection(
            loop, lambda: SerialReader(node), '/dev/ttyS5', baudrate=115200
        )
    except Exception as e:
        node.get_logger().error(f"No se pudo abrir el puerto serie: {e}")
        transport = None

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if transport:
            transport.close()

if __name__ == '__main__':
    asyncio.run(main_async())
