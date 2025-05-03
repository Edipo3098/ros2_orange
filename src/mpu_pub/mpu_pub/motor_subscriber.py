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
from robot_interfaces.msg import Anglemotor,MoveRobot
from robot_interfaces.msg import Command
import time
import serial


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('motor_subscriber')
        #self.subscription = self.create_subscription(Anglemotor,'motor_angles', self.listener_callback,10) # connect to motor control
        self.subscription = self.create_subscription(MoveRobot, 'motor_angles', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.publishers_ = self.create_publisher(Command, 'command_robot', 10)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.checkCommunication_Arduino()
        #self.checkCommunication_Arduino()

        self.msg_command = Command()
        self.msg_command.ready = True
        self.msg_command.armmoving = False
        self.msg_command.grippermoving = False
        self.msg_command.gripperopen = False
        self.msg_command.gripperclosed = False
        self.msg_command.quadmoving = False

        self.publishers_.publish(self.msg_command)
        self.isARM  = False
        self.isGait = False
        self.get_logger().info('Publish true')
        
    def checkCommunication_Arduino(self):
        serial_port = '/dev/ttyS5'
        baud_rate = 115200

        ser = serial.Serial(serial_port, baud_rate, timeout=1)

        # Send data over the serial connection
        ser.write(str('check').encode())
        ser.write(B"\n")
        time.sleep(0.2)
        received_data = ser.readline().decode().strip()
        self.get_logger().info('Received: "%s"' % received_data)
        
        if received_data == "check":
            self.get_logger().info('Communication with Arduino is OK')
            self.msg_command.ready = True
            self.publishers_.publish(self.msg_command)
            
        else:
            self.get_logger().info('Communication with Arduino is NOT OK')
            self.msg_command.ready = False
            self.publishers_.publish(self.msg_command)
            
    def timer_callback(self):
        self.publishers_.publish(self.msg_command)
        
  
    def listener_callback(self, msg):
        
        self.get_logger().info('Command "%s"' % msg.command)
        if msg.command == "ARM":
            self.get_logger().info('ARM')
            self.isARM = True
            
        elif msg.command == "m4":
            msg.command = "m3"
            self.get_logger().info('is publishing static gait')
            self.isGait = True
            
        else:
            msg.command = "origin"
            self.get_logger().info('is stoping gait')
            
            self.isARM = False
        serial_port = '/dev/ttyS5'
        baud_rate = 115200

        ser = serial.Serial(serial_port, baud_rate, timeout=1)

        counter = 0
        try:
            # Send data over the serial connection
            
            
            if ( self.isARM):
                ser.write(str(msg.command).encode())
                ser.write(B"\n")
                time.sleep(0.2)
                csv_line = f"{msg.m0},{msg.m1},{msg.m2},{msg.m3},{msg.m4},{2}\n"
                ser.write(csv_line.encode()) 
                self.isARM = False
                self.msg_command.armmoving = True
                self.msg_command.grippermoving = False
                self.msg_command.gripperopen = False
                self.msg_command.gripperclosed = False
                self.msg_command.quadmoving = False
            elif (self.isGait):
                ser.write(str(msg.command).encode())
                ser.write(B"\n")
                self.isGait = False
                self.msg_command.armmoving = False
                self.msg_command.grippermoving = False
                self.msg_command.gripperopen = False
                self.msg_command.gripperclosed = False
                self.msg_command.quadmoving = True
                
            else:
                ser.write(str(msg.command).encode())
                ser.write(B"\n")
                self.isARM = False    
            # Wait for a moment
            
            # Read response from the serial connection
            self.msg_command.ready = False
            self.publishers_.publish(self.msg_command )
            received_data = "False"
            while received_data != "True":
                received_data = ser.readline().decode().strip()
                self.get_logger().info('Received different than true: "%s"' % received_data)
                counter += 1
                if counter > 20:
                    self.get_logger().info('Received different than true: "%s"' % received_data)
                    
                    self.msg_command.ready = False
                    
                    break
                if received_data == "True":
                    self.get_logger().info('Received: "%s"' % received_data)
                    
                    self.msg_command.ready = True
                else:
                    
                    self.msg_command.ready = False
                    
            self.msg_command.ready =  received_data == "True"
            self.msg_command.armmoving = False
            self.msg_command.grippermoving = False
            self.msg_command.gripperopen = False
            self.msg_command.gripperclosed = False
            self.msg_command.quadmoving = False
            self.publishers_.publish(self.msg_command )

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
