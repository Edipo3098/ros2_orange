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
        #self.timer_communication = self.create_timer(2, self.checkCommunication_Arduino)
        self.timer_comm = self.create_timer(0.5, self.communicacion_arduino)
        self.Sending = False
        
        #self.checkCommunication_Arduino()
        self.robot_command = MoveRobot()    
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
        self.command_sended = False
        self.received_data = "False"
        self.get_logger().info('Publish true')
        self.serial_port = '/dev/ttyS5'
        self.baud_rate = 115200

        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
    def communicacion_arduino(self):
        if not self.Sending:
            self.get_logger().info('No new data"')
        if self.Sending == True:
            msg = self.robot_command
            self.get_logger().info('Received: "%s"' % msg)
            self.get_logger().info('Command "%s"' % msg.command)
            self.isARM = False
            self.isGait = False
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
            

            counter = 0
            try:
                # Send data over the serial connection
                if not self.command_sended:
                    if ( self.isARM):
                        self.ser.write(str(msg.command).encode())
                        self.ser.write(B"\n")
                    
                        csv_line = f"{int(msg.m0)},{int(msg.m1)},{int(msg.m2)},{int(msg.m3)},{int(msg.m4)},{int(msg.m5)}\n"
                        self.ser.write(csv_line.encode()) 
                        self.get_logger().info('Sended: "%s"' % csv_line)
                        self.msg_command.armmoving = True
                        self.msg_command.grippermoving = False
                        self.msg_command.gripperopen = False
                        self.msg_command.gripperclosed = False
                        self.msg_command.quadmoving = False
                        self.command_sended = True
                    elif (self.isGait):
                        csv_line = f"{msg.command}\n"
                        self.ser.write(csv_line.encode()) 
                        self.command_sended = True
                        #self.ser.write(str(msg.command).encode())
                        #self.ser.write(B"\n")
                        
                        self.msg_command.armmoving = False
                        self.msg_command.grippermoving = False
                        self.msg_command.gripperopen = False
                        self.msg_command.gripperclosed = False
                        self.msg_command.quadmoving = True
                        
                    else:
                        self.ser.write(str(msg.command).encode())
                        self.ser.write(B"\n")
                        self.get_logger().info('Sended: "%s"' % msg.command)
                        self.command_sended = True
                    
                # Wait for a moment
                
                # Read response from the serial connection
                self.msg_command.ready = False
                self.publishers_.publish(self.msg_command )
                
                if self.received_data  != "True" and self.command_sended:
                    try:
                        self.received_data  = self.ser.readline().decode().strip()
                        
                    except UnicodeDecodeError as e:
                        self.get_logger().warn(f"Error decoding {self.received_data !r}: {e}")
                        
                    self.get_logger().info('Waiting callback true: "%s"' % self.received_data)
                    
                        
                    if self.received_data  == "True":
                        self.get_logger().info('Received: "%s"' % self.received_data )
                        self.command_sended = False
                        self.msg_command.ready = True
                        self.Sending = False
                    else:
                        
                        self.msg_command.ready = False
                elif self.received_data  == "True" and self.command_sended:
                    self.get_logger().info('Received: "%s"' % self.received_data )
                    self.msg_command.ready = False
                    self.Sending = False
                    self.command_sended = False
                    self.received_data  = "False"
                self.ser.reset_input_buffer()   

            except KeyboardInterrupt:
                # If Ctrl+C is pressed, break out of the loop
                print("Keyboard interrupt detected. Exiting...")
            finally:
                # Close the serial port, even if an exception occurs
                self.ser.reset_input_buffer()
                self.ser.close()
                #self.timer_communication.cancel()
            
        #self.timer_communication = self.create_timer(2, self.checkCommunication_Arduino)
           
    def checkCommunication_Arduino(self):
        
        try:
            serial_port = '/dev/ttyS5'
            baud_rate = 115200

            self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
            if self.Sending:
                return
            

            # Send data over the serial connection
            #self.ser.write(str('check\n').encode())
            #self.ser.write(B"\n")
            
            csv_line = f"{'check'}\n"
            self.ser.write(csv_line.encode()) 
            
            try:
                received_data = self.ser.readline().decode().strip()
                    
            except UnicodeDecodeError as e:
                self.get_logger().warn(f"Error decoding {received_data!r}: {e}")
                received_data = received_data.decode('utf-8', errors='ignore').strip()
            self.get_logger().info('Received: "%s"' % received_data)
            
            if received_data == "check":
                self.get_logger().info('Communication with Arduino is OK')
                self.msg_command.ready = True
                self.publishers_.publish(self.msg_command)
                
            else:
                self.get_logger().info('Communication with Arduino is NOT OK')
                self.ser.reset_input_buffer()
        except KeyboardInterrupt:
            # If Ctrl+C is pressed, break out of the loop
            print("Keyboard interrupt detected. Exiting...")
        finally:
            # Close the serial port, even if an exception occurs
            self.ser.reset_input_buffer()
            self.ser.close()
            
    def timer_callback(self):
        self.publishers_.publish(self.msg_command)
        
  
    def listener_callback(self, msg):
        self.Sending = True
        self.robot_command = msg
        
        


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
