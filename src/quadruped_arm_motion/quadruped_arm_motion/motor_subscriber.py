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
        self.isARM  = False
        self.get_logger().info('Publish true')
        
  
    def listener_callback(self, msg):
        
        self.get_logger().info('SOMETHING HERE')
        if msg.robot == "ARM":
            self.get_logger().info('is publishing ARM')
            self.get_logger().info('Joint "%s"' % msg.joint)
            self.get_logger().info('Z0 "%s"' % msg.angle)
            self.isARM = True
            
        else:
            self.get_logger().info('is publishing QUAD')
            self.get_logger().info('Leg "%s"' % msg.leg)
            self.get_logger().info('Joint "%s"' % msg.joint)
            self.get_logger().info('P0 Z0 "%s"' % msg.angle)
            self.isARM = False
                

        #time.sleep(5)
        msg_command = Command()
        msg_command.ready = False
        self.publishers_.publish(msg_command)

        


        serial_port = '/dev/ttyS5'
        baud_rate = 115200

        ser = serial.Serial(serial_port, baud_rate, timeout=1)


        try:
            # Send data over the serial connection
            
            
            if ( self.isARM):
                
                ser.write(str(msg.robot).encode())
                ser.write(B"\n")
                time.sleep(0.5)
                ser.write(str(msg.joint).encode())
                ser.write(B"\n")
                time.sleep(0.5)
                ser.write(str(msg.angle).encode())
                ser.write(B"\n")
                time.sleep(0.5)
                ser.write(str(2).encode())
                ser.write(B"\n")
                time.sleep(0.5)
            else:
                ser.write(str(msg.robot).encode())
                ser.write(B"\n")
                time.sleep(0.5)
                ser.write(str(msg.leg).encode())
                ser.write(B"\n")
                time.sleep(0.5)
                ser.write(str(msg.joint).encode())
                ser.write(B"\n")
                time.sleep(0.5)
                ser.write(str(msg.angle).encode())
                ser.write(B"\n")
                time.sleep(0.5)
                ser.write(str(2).encode())
                ser.write(B"\n")
                time.sleep(0.5)
            # Wait for a moment
            
            # Read response from the serial connection
            received_data = "False"
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
