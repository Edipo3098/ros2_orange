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
####### NOT USED ############

import rclpy
from rclpy.node import Node

import time
from std_msgs.msg import String
from robot_interfaces.msg import Anglemotor
from robot_interfaces.msg import Command



class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('quadruped_publisher')
        self.Subscription_trayectory = self.create_subscription(Anglemotor, 'trayectory',self.command_callback, 10)
        self.publisher_ = self.create_publisher(Anglemotor, 'motor_angles', 10)
        #self.subscription = self.create_subscription(Command, 'command_robot', self.command_callback, 10)
        #self.Check_communication()
        #timer_period = 5 # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.movement = True

    def send_orange(self,msg):
        
        if msg.message == "M1":
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

       
        message = Anglemotor()
        message = msg
        self.publisher_.publish(message)
    
    def command_callback(self, msg):
        self.send_orange(msg)


      



def main(args=None):
    rclpy.init(args=args)

    quadruped_publisher = MinimalPublisher()

    rclpy.spin(quadruped_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    quadruped_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
