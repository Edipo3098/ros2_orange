
import rclpy
from rclpy.node import Node

import time
from std_msgs.msg import String
from robot_interfaces.msg import Anglemotor
from robot_interfaces.msg import Command
import numpy as np

arm = np.zeros((5, 6))
arm[0] = [0, 0, 0, 0, 0, 0]
arm[1] = [90, 100, 0, 0, 0, 0]
arm[2] = [90, 100, 45, 0, 0,1]
arm[3] = [0, 100, 45, 45, 0,0]
arm[4] = [0, 100, 45, 45, 90,1]

quad = np.zeros((5, 12))
quad[0] = [0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0]
quad[1] = [1 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0]
quad[2] = [2 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0]
quad[3] = [3 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0]
quad[4] = [4 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0]

class RobotKynematics:

    def __init__(self):
        super().__init__()
        self.IsArmMovement = False
        self.armJoints = np.zeros((5))
        self.legJoints = np.zeros((4, 3))
        self.COG_pos = np.zeros(3)
        self.COG_or = np.zeros(3)
        self.L1 =  5/100
        self.L2 = 25/100
        self.L3 = 25/100
        self.a = 0.6
        self.b = 0.2
        self.z_cog_offset = 0.4037
        self.HomeArmJoint = np.zeros(3)
        self.HomeQuadJoint = np.array([0,np.pi/4,-np.pi/2])
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('robotTrayectory')
        self.trayectory_pub= self.create_publisher(Anglemotor, 'motor_angles', 10)
        self.trayectory_sub = self.create_subscription(Anglemotor, 'matlab', self.timer_callback, 10)

        
        #self.Check_communication()
        #timer_period = 5 # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.changeData = False
        self.movement = True

    def timer_callback(self,msg):

        self.trayectory_pub.publish(msg)
        self.get_logger().info('Publish angles')   
        
        

        

        '''        msg.p0z0 = float(quad[self.i][0])
        msg.p0z1 = float(quad[self.i][1])
        msg.p0z2 = float(quad[self.i][2])
        msg.p1z0 = float(quad[self.i][3])
        msg.p1z1 = float(quad[self.i][4])
        msg.p1z2 = float(quad[self.i][5])
        msg.p2z0 = float(quad[self.i][6])
        msg.p2z1 = float(quad[self.i][7])
        msg.p2z2 = float(quad[self.i][8])
        msg.p3z0 = float(quad[self.i][9])
        msg.p3z1 = float(quad[self.i][10])
        msg.p3z2 = float(quad[self.i][11])

        msg.armz0 = float(arm[self.i][0])
        msg.armz1 = float(arm[self.i][1])
        msg.armz2 = float(arm[self.i][2])
        msg.armz3 = float(arm[self.i][3])
        msg.armz4 = float(arm[self.i][4])
        msg.gripper =  float(arm[self.i][5])'''

        
        

    
    def command_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.ready)
        if msg.ready == True:
            self.get_logger().info('Starting the robot')
            self.timer_callback()
        elif msg.ready == False:
            self.get_logger().info('Stopping the robot')
        else:
            self.get_logger().info('Command not recognized')

      



def main(args=None):
    rclpy.init(args=args)

    trayectory_planning= MinimalPublisher()

    rclpy.spin(trayectory_planning)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    trayectory_planning.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()