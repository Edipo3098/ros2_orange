# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Anglemotor
from robot_interfaces.msg import Command
import time
import serial


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('motor_subscriber')
        #self.subscription = self.create_subscription(Anglemotor,'motor_angles', self.listener_callback,10) # connect to motor control
        self.subscription = self.create_subscription(Anglemotor, 'motor_angles', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.publishers_ = self.create_publisher(Command, 'command_robot', 10)
        #self.checkCommunication_Arduino()

        msg_command = Command()
        msg_command.ready = True
        self.publishers_.publish(msg_command)
        self.get_logger().info('Publish true')
        
  
    def listener_callback(self, msg):
        
        self.get_logger().info('SOMETHING HERE')
        if msg.message == "m1":
            self.get_logger().info('is publishing ARM')
            self.get_logger().info('Z0 "%s"' % msg.armz0)
            self.get_logger().info('Z1 "%s"' % msg.armz1)
            self.get_logger().info('Z2 "%s"' % msg.armz2)
            self.get_logger().info('Z3 "%s"' % msg.armz3)
            self.get_logger().info('Z4 "%s"' % msg.armz4)
            self.get_logger().info('Gripper "%s"' % msg.gripper)
        else:
            self.get_logger().info('is publishing QUAD')
            self.get_logger().info('P0 Z0 "%s"' % msg.p0z0)
            self.get_logger().info('P0 Z1 "%s"' % msg.p0z1)
            self.get_logger().info('P0 Z2 "%s"' % msg.p0z2)
            self.get_logger().info('P1 Z0 "%s"' % msg.p1z0)
            self.get_logger().info('P1 Z1 "%s"' % msg.p1z1)
            self.get_logger().info('P1 Z2 "%s"' % msg.p1z2)
            self.get_logger().info('P2 Z0 "%s"' % msg.p2z0)
            self.get_logger().info('P2 Z1 "%s"' % msg.p2z1)
            self.get_logger().info('P2 Z2 "%s"' % msg.p2z2)
            self.get_logger().info('P3 Z0 "%s"' % msg.p3z0)
            self.get_logger().info('P3 Z1 "%s"' % msg.p3z1)
            self.get_logger().info('P3 Z2 "%s"' % msg.p3z2)            

        time.sleep(5)
        msg_command = Command()
        msg_command.ready = False
        self.publishers_.publish(msg_command)

        time.sleep(5)
        msg_command.ready = True
        self.publishers_.publish(msg_command)


        serial_port = '/dev/ttyS5'
        baud_rate = 115200

        ser = serial.Serial(serial_port, baud_rate, timeout=1)


        try:
            # Send data over the serial connection
            
            
            if ( msg.message == "m1"):
                data_to_send = "m1"
                ser.write(data_to_send.encode())  # Encode string as bytes before sending
                ser.write(B"\n")
                time.sleep(0.5)
                ser.write(str(msg.armz0).encode())
                ser.write(B"\n")
                time.sleep(0.5)
                ser.write(str(msg.armz1).encode())
                ser.write(B"\n")
                time.sleep(0.5)
                ser.write(str(msg.armz2).encode())
                ser.write(B"\n")
                time.sleep(0.5)
                ser.write(str(msg.armz3).encode())
                ser.write(B"\n")
                time.sleep(0.5)
                ser.write(str(msg.armz4).encode())
                ser.write(B"\n")
                time.sleep(0.5)
                ser.write(str(2).encode())
                ser.write(B"\n")
                time.sleep(0.5)
            else:
                data_to_send = "m2"
                ser.write(data_to_send.encode())  # Encode string as bytes before sending
                ser.write(B"\n")
                time.sleep(0.5)
                ser.write(str(msg.p0z0).encode())
                ser.write(B"\n")
                time.sleep(0.5)
                ser.write(str(msg.p0z1).encode())
                ser.write(B"\n")
                time.sleep(0.5)
                ser.write(str(msg.p0z2).encode())
                ser.write(B"\n")
                time.sleep(0.5)
                ser.write(str(msg.p1z0).encode())
                ser.write(B"\n")
                time.sleep(0.5)
                ser.write(str(msg.p1z1).encode())
                ser.write(B"\n")
                time.sleep(0.5)
                ser.write(str(msg.p1z2).encode())
                ser.write(B"\n")
                time.sleep(0.5)
                ser.write(str(msg.p2z0).encode())
                ser.write(B"\n")
                time.sleep(0.5)
                ser.write(str(msg.p2z1).encode())
                ser.write(B"\n")
                time.sleep(0.5)
                ser.write(str(msg.p2z2).encode())
                ser.write(B"\n")
                time.sleep(0.5)
                ser.write(str(msg.p3z0).encode())
                ser.write(B"\n")
                time.sleep(0.5)
                ser.write(str(msg.p3z1).encode())
                ser.write(B"\n")
                time.sleep(0.5)
                ser.write(str(msg.p3z2).encode())
                ser.write(B"\n")
                time.sleep(0.5)
                ser.write(str(2).encode())
            # Wait for a moment
            
            # Read response from the serial connection
            received_data = ser.readline().decode().strip()
            while received_data != "True":
                received_data = ser.readline().decode().strip()
                self.get_logger().info('Received different than true: "%s"' % received_data)
                msg_command = Command()
                msg_command.ready = False
                self.publishers_.publish(msg_command)
            if received_data == "True":
                self.get_logger().info('Received: "%s"' % received_data)
                msg_command = Command()
                msg_command.ready = True
                self.publishers_.publish(msg_command)
            else:
                msg_command = Command()
                msg_command.ready = False
                self.publishers_.publish(msg_command)

            self.get_logger().info('Received wrong:  "%s"' % received_data)

        except KeyboardInterrupt:
            # If Ctrl+C is pressed, break out of the loop
            print("Keyboard interrupt detected. Exiting...")
        finally:
            # Close the serial port, even if an exception occurs
            ser.close()
        
        


def main(args=None):
    rclpy.init(args=args)

    motor_subscriber = MinimalSubscriber()

    rclpy.spin(motor_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
