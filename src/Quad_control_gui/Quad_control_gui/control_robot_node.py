import sys
from threading import Thread
from enum import Enum
import copy

from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QWidget, QStackedWidget,QVBoxLayout,QHBoxLayout
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.patches import Circle
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor,QPalette
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QFont, QColor
import numpy as np

import rclpy
from rclpy.node import Node

from Quad_control_gui.MyMplCanvas import MyMplCanvas,IndicatorWidget
from Quad_control_gui.control_robot import Ui_RobotControl
from collections import deque
import pyqtgraph as pg
from tf2_ros import TransformListener, Buffer

from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String,Bool

from robot_interfaces.msg import Anglemotor
from robot_interfaces.msg import Command,MoveRobot
from robot_interfaces.msg import Mpu,COGframe
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
ARM = 0
QUAD = 1

FL = 0
FR = 1
BR = 2
BL = 3

Joint_0 = 0
Joint_1 = 1
Joint_2 = 2
Joint_3 = 3
Joint_4 = 4
Joint_5 = 5
class Joints(Enum):
    
    frontLeft_hip_motor_joint = 'frontLeft_hip_motor_joint'
    frontLeft_knee_joint = 'frontLeft_knee_joint'
    frontLeft_ankle_joint = 'frontLeft_ankle_joint'
    frontRight_hip_motor_joint = 'frontRight_hip_motor_joint'
    frontRight_knee_joint = 'frontRight_knee_joint'
    frontRight_ankle_joint = 'frontRight_ankle_joint'
    backRight_hip_motor_joint = 'backRight_hip_motor_joint'
    backRight_knee_joint = 'backRight_knee_joint'
    backRight_ankle_joint = 'backRight_ankle_joint'
    backLeft_hip_motor_joint = 'backLeft_hip_motor_joint'
    backLeft_knee_joint = 'backLeft_knee_joint'
    backLeft_ankle_joint = 'backLeft_ankle_joint'
    articulacion1 = 'articulacion1'
    articulacion2 = 'articulacion2'
    articulacion3 = 'articulacion3'
    articulacion4 = 'articulacion4'
    articulacion5 = 'articulacion5'


class MyNode(Node):
    def __init__(self):
        super().__init__('control_robot_node')
        
        self.data1 = 0
        self.data2 = 0
        self.msg_move = Anglemotor()
        self.msg_ik = Bool()
        self.msg_move_robot = MoveRobot()
        self.publisher = self.create_publisher(Anglemotor, 'motor_angles2', 10)
        self.publisher_commandRobot = self.create_publisher(MoveRobot, 'motor_angles', 10)
        self.subscription = self.create_subscription(Command, 'command_robot', self.listener_callback, 10)
        self.joint_states_pub = self.create_publisher(JointState,'joint_states',10)
        self.subscription_cog = self.create_subscription(COGframe, 'kalman_cog_frame_3', self.cog_callback, 10)
        self.subscription_IkSolution  = self.create_subscription(JointTrajectory, '/arm_trajectory', self.ikCallback, 10)
        self.publisher_getIkSolution = self.create_publisher(Bool, 'calculate_Ik', 10)
        self.getIk = False
        self.msg_move_robot.m0 = float(0)
        self.msg_move_robot.m1 = float(0)
        self.msg_move_robot.m2 = float(0)
        self.msg_move_robot.m3 = float(0)
        self.msg_move_robot.m4 = float(0)
        self.msg_move_robot.m5 = float(0)
        self.sendCogReady = False
        self.countCogmsg = 0
        self.IkSolution = np.zeros(5, dtype=float)  # Initialize IK solution array
        # Sample data storage for x, y, z, roll, pitch, yaw with 3 sources for each
        self.max_len = 500  # o 200, dependiendo de cuánto quieras ver “en pantalla”
        self.data_x     = deque(maxlen=self.max_len )
        self.data_y     = deque(maxlen=self.max_len )
        self.data_z     = deque(maxlen=self.max_len )
        self.data_roll  = deque(maxlen=self.max_len )
        self.data_pitch = deque(maxlen=self.max_len )
        self.data_yaw   = deque(maxlen=self.max_len )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.footPose = [[0,0],[0,0],[0,0],[0,0]]
        self.footPose_k = [[0,0],[0,0],[0,0],[0,0]]
        self.endEfector =   [0,0,0,0,0,0]
        self.aprilTagPose = [0,0,0,0,0,0]
        self.ikSol = np.zeros(5, dtype=float)
        self.cogPose = [0,0,0]
        self.cogOR = [0,0,0]
        self.COG = np.append(self.cogPose,self.cogOR)
        

       
        # Create Arm objests
        self.armJoint = np.array([0.0,0.0,0.0,0.0,0.0])
        
        self.legJoints_FL = np.array([0.0 , 0.0 , 0.0])
        self.legJoints_FR = copy.deepcopy(self.legJoints_FL)  # Creates an independent list
        self.legJoints_BL = copy.deepcopy(self.legJoints_FL)
        self.legJoints_BR = copy.deepcopy(self.legJoints_FL)

        

         # Timer to periodically check the transform
        self.timer_footPose = self.create_timer(0.5, self.timer_foot_callback)
        self.timer_foot_callback()

        
        self.joint_state_msg = JointState()
         # List of joint names from your URDF
        self.joint_state_msg.name = self.joint_state_msg.name = [joint.value for joint in Joints]
        
        self.joint_state_msg.position = [value for value in len(Joints)* [0.0]]  # Initialize positions to 0.0 for each joint

        # Set the current time for the header
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        self.RobotReady = False
        self.robotMoving = False
        self.armMoving = False
        # Publish the initial joint states
        
        self.get_logger().info('Initial joint states published.')
        self.arm_joints = np.zeros(5, dtype=float)

        # Y para cada pata, un arreglo de 3 ángulos (por ejemplo: cadera, rodilla, tobillo)
        self.leg_joints_FL = np.array([0.0, 0.0, 15], dtype=float)  # Front Left leg joints
        self.leg_joints_FR = np.array([0.0, 0.0, 15], dtype=float)
        self.leg_joints_BL = np.array([0.0, 0.0, 15], dtype=float)
        self.leg_joints_BR = np.array([0.0, 0.0, 15], dtype=float)
        self.joint_states_pub.publish(self.joint_state_msg)
    def ikCallback(self, msg):
        """
        Callback function triggered when a message is received on the 'arm_trajectory' topic.
        The label in the UI will be updated with the message data.
        """
        
        if len(msg.points) == 0:
            self.get_logger().info('No hay puntos en el mensaje JointTrajectory.')
        else:
            for idx, point in enumerate(msg.points):
                pt = msg.points[0]

                
                self.ikSol[:] = pt.positions
                
    def add_data(self, msg, index):
        """Simplemente agrega al deque; automáticamente descarta lo más antiguo."""
        self.data_x.append(msg.pos_x)
        self.data_y.append(msg.pos_y)
        self.data_z.append(msg.pos_z)
        self.data_roll.append(msg.roll)
        self.data_pitch.append(msg.pitch)
        self.data_yaw.append(msg.yaw)
        

        

    def cog_callback(self, msg):
        """Callback for cog_data topic."""
        self.countCogmsg += 1
        if self.countCogmsg > 20:
            self.sendCogReady = True
            self.countCogmsg = 0
        else:
            self.sendCogReady = False
            
        if self.sendCogReady:
            self.add_data(msg, 0)
    def timer_foot_callback(self):
        foot_joint= ['frontLeft_foot','frontRight_foot','backRight_foot','backLeft_foot'] 
        arm_joint= ['endEfector'] 
        x = 0
        for i in foot_joint:
            
            try:
                
                # Look for the transform from the world frame to the end effector
                now = rclpy.time.Time()
                transform = self.tf_buffer.lookup_transform('map', i, now)
                
                # Extract the translation (position) and rotation (orientation)
                trans = transform.transform.translation
                rot = transform.transform.rotation  
                
                # Print the position and orientation of the end effector
                #self.get_logger().info(f"{i}: x={trans.x}, y={trans.y}, z={trans.z}")
                
                self.footPose[x] = [trans.x ,trans.y]
               

                x +=1
            
            except Exception as e:
                pass
                #self.get_logger().warn(f"Could not get transform: {str(e)}")
        
       
        
       

        
        """
        self.get_logger().info(f"foot_LF,: x={foot_LF[0]}, y={foot_LF[1]}, z={foot_LF[2]}")
        self.get_logger().info(f"foot_RF,: x={foot_RF[0]}, y={foot_RF[1]}, z={foot_RF[2]}")
        self.get_logger().info(f"foot_RB,: x={foot_RB[0]}, y={foot_RB[1]}, z={foot_RB[2]}")
        self.get_logger().info(f"foot_LB,: x={foot_LB[0]}, y={foot_LB[1]}, z={foot_LB[2]}")
        
        """
            # Look for the transform from the world frame to the end effector
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform('map', 'base_link', now)
            
            # Extract the translation (position) and rotation (orientation)
            trans = transform.transform.translation
            rot = transform.transform.rotation  
            
            # Print the position and orientation of the end effector
            #self.get_logger().info(f"Yaw_body,: x={trans.x}, y={trans.y}, z={trans.z}")
            
            self.cogPose = [trans.x ,trans.y,trans.z]
            self.cogOR =    [rot.x , rot.y,  rot.z]
            self.COG = np.append(self.cogPose,self.cogOR)
            self.R_Front.changePoseLeg(self.COG)
            self.R_Back.changePoseLeg(self.COG)
            self.L_Front.changePoseLeg(self.COG)
            self.L_Back.changePoseLeg(self.COG)
            
        except Exception as e:
            pass
            #self.get_logger().warn(f"Could not get transform: {str(e)}")
        try:
                
                # Look for the transform from the world frame to the end effector
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform('base_link', 'endEfector', now)
            
            # Extract the translation (position) and rotation (orientation)
            trans = transform.transform.translation
            rot = transform.transform.rotation  
            
            # Print the position and orientation of the end effector
            
                
            self.endEfector = [trans.x ,trans.y,trans.z,rot.x,rot.y,rot.z]
            transform = self.tf_buffer.lookup_transform('base_link', 'tag36h11:5', now)
             # Extract the translation (position) and rotation (orientation)
            trans = transform.transform.translation
            rot = transform.transform.rotation  
            #self.get_logger().info(f"AprilTag: x={trans.x}, y={trans.y}, z={trans.z}")
            # Print the position and orientation of the end effector
            #self.get_logger().info(f"End Effecor: x={trans.x}, y={trans.y}, z={trans.z}")
                
            self.aprilTagPose = [trans.x ,trans.y,trans.z,rot.x,rot.y,rot.z]
            #self.get_logger().info(f"End Effecor kinematics: x={endPose[0]}, y={endPose[1]}, z={endPose[2]}")

            #self.get_logger().info(f"Rot x={rot.x}, y={rot.y}, z={rot.z}")
            #self.get_logger().info(f"Rot kinematics: x={orientation[0]}, y={orientation[1]}, z={orientation[2]}")  

            
            
        except Exception as e:
            pass
            #self.get_logger().warn(f"Could not get transform: {str(e)}")

        
    def listener_callback(self, msg):
        """
        Callback function triggered when a message is received on the 'mpu_data' topic.
        The label in the UI will be updated with the message data.
        """
        
        self.RobotReady = msg.ready
        self.robotMoving = msg.quadmoving
        self.armMoving = msg.armmoving
        
        
    def publish_message(self, message):
        """
        Publishes a string message to the 'output_topic'.
        """
        
        self.publisher.publish(self.msg_move)
        self.msg_move_robot.m0 =  self.arm_joints[0]  #float(self.arm.getJoint(Joint_0)),  # Update ARM
        self.msg_move_robot.m1 =  self.arm_joints[1]  #float(self.arm.getJoint(Joint_1)),  # Update 'joint_0' of the FL leg
        self.msg_move_robot.m2 =  self.arm_joints[2]  #float(self.arm.getJoint(Joint_2)),  # Update 'joint_0' of the FL leg
        self.msg_move_robot.m3 =  self.arm_joints[3]  #float(self.arm.getJoint(Joint_3)) ,  # Update 'joint_0' of the FL leg
        self.msg_move_robot.m4 =  self.arm_joints[4]  #float(self.arm.getJoint(Joint_4))  # Update 'joint_0' of the FL leg
        self.publisher_commandRobot.publish(self.msg_move_robot)
    def publish_joint_states(self):
        """
        Publishes joint states to the /joint_states topic
        """
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        # Here you update the joint states with the actual positions
        # self.joint_state_msg.position[0] = <your_value>

        # Publish the joint states
        self.joint_states_pub.publish(self.joint_state_msg)



class MyWindow(QMainWindow):
    def __init__(self, ros_node):
        super(MyWindow, self).__init__()
        self.resize(1200, 800)  # Set the size of the main window to be bigger


        self.ui = Ui_RobotControl()  # Instantiate the UI class generated from the .ui file
        # Create a central widget to hold your stacked widget
        
        stacked_widget = QStackedWidget()  # This should be a QStackedWidget, not a generic QWidget
        self.ui.setupUi(stacked_widget)
        # Set the QStackedWidget as the central widget of the QMainWindow
        self.setCentralWidget(stacked_widget)  # THIS IS THE CRUCIAL LINE

        self.stacked_widget = stacked_widget  
        pal = QPalette()
        pal.setColor(QPalette.Window, Qt.white)
        self.setPalette(pal)
        self.stacked_widget
        self.stacked_widget.setCurrentIndex(0)
        self.ui.controlBasic.setVisible(True)
        self.stacked_widget.update()

        # Add the indicator widget to the layout
        self.indicator_robot = IndicatorWidget()  # Create the indicator widget
        self.indicator_label = QLabel("Robot Status:")  # Create a label for the indicator
        self.indicator_robot_2 = IndicatorWidget()  # Create the indicator widget
        self.indicator_label_2 = QLabel("Robot Status:")  # Create a label for the indicator
        self.indicator_robot_3 = IndicatorWidget()  # Create the indicator widget
        self.indicator_label_3 = QLabel("Robot Status:")  # Create a label for the indicator
         # Assuming you have a placeholder in the .ui file
        self.ui.layout_indicator = QHBoxLayout(self.ui.quadMoving)  # Assuming you have a placeholder in the .ui file
        self.ui.layout_indicator_page2 = QHBoxLayout(self.ui.quadMoving_2) 
        self.ui.layout_indicator_page3 = QHBoxLayout(self.ui.quadMoving_3) 
        
        self.ui.layout_indicator.addWidget(self.indicator_label)
        self.ui.layout_indicator.addWidget(self.indicator_robot) 
        self.ui.layout_indicator_page2.addWidget(self.indicator_label_2)
        self.ui.layout_indicator_page2.addWidget(self.indicator_robot_2) 
         
        self.ui.layout_indicator_page3.addWidget(self.indicator_label_3)
        self.ui.layout_indicator_page3.addWidget(self.indicator_robot_3)  

        self.indicator_arm= IndicatorWidget()  # Create the indicator widget
        self.indicator_label_arm = QLabel("Arm moving:")  # Create a label for the indicator
        self.indicator_arm_2= IndicatorWidget()  # Create the indicator widget
        self.indicator_label_arm_2 = QLabel("Arm moving:")  # Create a label for the indicator
        self.indicator_arm_3= IndicatorWidget()  # Create the indicator widget
        self.indicator_label_arm_3 = QLabel("Arm moving:")  # Create a label for the indicator
        
        self.ui.layout_indicator2 = QHBoxLayout(self.ui.armMoving)  # Assuming you have a placeholder in the .ui file
        self.ui.layout_indicator2_page2 = QHBoxLayout(self.ui.armMoving_2) 
        self.ui.layout_indicator2_page3 = QHBoxLayout(self.ui.armMoving_3) 
        self.ui.layout_indicator2.addWidget(self.indicator_label_arm)
        self.ui.layout_indicator2.addWidget(self.indicator_arm)  
        self.ui.layout_indicator2_page2.addWidget(self.indicator_label_arm_2)
        self.ui.layout_indicator2_page2.addWidget(self.indicator_arm_2)  
        self.ui.layout_indicator2_page3.addWidget(self.indicator_label_arm_3)
        self.ui.layout_indicator2_page3.addWidget(self.indicator_arm_3)  
        
        self.indicator_leg = IndicatorWidget()  # Create the indicator widget
        self.indicator_label_leg = QLabel("Quad moving:")  # Create a label for the indicator
        self.indicator_leg_2 = IndicatorWidget()  # Create the indicator widget
        self.indicator_label_leg_2 = QLabel("Quad moving:")  # Create a label for the indicator
        self.indicator_leg_3 = IndicatorWidget()  # Create the indicator widget
        self.indicator_label_leg_3 = QLabel("Quad moving:")  # Create a label for the indicator
        
        self.ui.layout_indicator3 = QHBoxLayout(self.ui.LegMoving)  # Assuming you have a placeholder in the .ui file
        self.ui.layout_indicator3_page2 = QHBoxLayout(self.ui.LegMoving_2) 
        self.ui.layout_indicator3_page3 = QHBoxLayout(self.ui.LegMoving_3) 
        self.ui.layout_indicator3.addWidget(self.indicator_label_leg)
        self.ui.layout_indicator3.addWidget(self.indicator_leg)  
        self.ui.layout_indicator3_page2.addWidget(self.indicator_label_leg_2)
        self.ui.layout_indicator3_page2.addWidget(self.indicator_leg_2)  
        self.ui.layout_indicator3_page3.addWidget(self.indicator_label_leg_3)
        self.ui.layout_indicator3_page3.addWidget(self.indicator_leg_3)  
        # Joint angles Inverse Kinematics
        self.indicator_j0 = QLabel("Joint 0:")  # Create a label for the indicator
        self.indicator_j0_value = QLabel("")  # Create a label for the indicato
        self.ui.layout_indicator4 = QHBoxLayout(self.ui.jointAngles_0)  # Assuming you have a placeholder in the .ui file
        self.ui.layout_indicator4.addWidget(self.indicator_j0)
        self.ui.layout_indicator4.addWidget(self.indicator_j0_value)  
        
        # Joint angles Inverse Kinematics
        self.indicator_j1 = QLabel("Joint 1:")  # Create a label for the indicator
        self.indicator_j1_value = QLabel("")  # Create a label for the indicato
        self.ui.layout_indicator5 = QHBoxLayout(self.ui.jointAngles_1)  # Assuming you have a placeholder in the .ui file
        self.ui.layout_indicator5.addWidget(self.indicator_j1)
        self.ui.layout_indicator5.addWidget(self.indicator_j1_value) 
        # Joint angles Inverse Kinematics
        self.indicator_j2 = QLabel("Joint 2:")  # Create a label for the indicator
        self.indicator_j2_value = QLabel("")  # Create a label for the indicato
        self.ui.layout_indicator6 = QHBoxLayout(self.ui.jointAngles_2)  # Assuming you have a placeholder in the .ui file
        self.ui.layout_indicator6.addWidget(self.indicator_j2)
        self.ui.layout_indicator6.addWidget(self.indicator_j2_value)
        # Joint angles Inverse Kinematics
        self.indicator_j3 = QLabel("Joint 1:")  # Create a label for the indicator
        self.indicator_j3_value = QLabel("")  # Create a label for the indicato
        self.ui.layout_indicator7 = QHBoxLayout(self.ui.jointAngles_3)  # Assuming you have a placeholder in the .ui file
        self.ui.layout_indicator7.addWidget(self.indicator_j3)
        self.ui.layout_indicator7.addWidget(self.indicator_j3_value)
        # Joint angles Inverse Kinematics
        self.indicator_j4 = QLabel("Joint 1:")  # Create a label for the indicator
        self.indicator_j4_value = QLabel("")  # Create a label for the indicato
        self.ui.layout_indicator8 = QHBoxLayout(self.ui.jointAngles_4)  # Assuming you have a placeholder in the .ui file
        self.ui.layout_indicator8.addWidget(self.indicator_j4)
        self.ui.layout_indicator8.addWidget(self.indicator_j4_value) 
        
        # AprilTagLocation
        self.indicator_apX = QLabel("Pos X:")  # Create a label for the indicator
        self.indicator_apX_value = QLabel("")  # Create a label for the indicato
        self.ui.layout_indicator9 = QHBoxLayout(self.ui.tag5_x)  # Assuming you have a placeholder in the .ui file
        self.ui.layout_indicator9.addWidget(self.indicator_apX)
        self.ui.layout_indicator9.addWidget(self.indicator_apX_value)  
        # Joint angles Inverse Kinematics
        self.indicator_apY = QLabel("Pos Y:")  # Create a label for the indicator
        self.indicator_apY_value = QLabel("")  # Create a label for the indicato
        self.ui.layout_indicator10 = QHBoxLayout(self.ui.tag5_y)  # Assuming you have a placeholder in the .ui file
        self.ui.layout_indicator10.addWidget(self.indicator_apY)
        self.ui.layout_indicator10.addWidget(self.indicator_apY_value)  
        # Joint angles Inverse Kinematics
        self.indicator_apZ = QLabel("Pos Z:")  # Create a label for the indicator
        self.indicator_apZ_value = QLabel("")  # Create a label for the indicato
        self.ui.layout_indicator11 = QHBoxLayout(self.ui.tag5_z)  # Assuming you have a placeholder in the .ui file
        self.ui.layout_indicator11.addWidget(self.indicator_apZ)
        self.ui.layout_indicator11.addWidget(self.indicator_apZ_value) 
        # Joint angles Inverse Kinematics
        self.indicator_apROl = QLabel("Roll:")  # Create a label for the indicator
        self.indicator_apROl_value = QLabel("")  # Create a label for the indicato
        self.ui.layout_indicator12 = QHBoxLayout(self.ui.tag5_roll)  # Assuming you have a placeholder in the .ui file
        self.ui.layout_indicator12.addWidget(self.indicator_apROl)
        self.ui.layout_indicator12.addWidget(self.indicator_apROl_value)
        # Joint angles Inverse Kinematics
        self.indicator_apPit = QLabel("Pitch:")  # Create a label for the indicator
        self.indicator_apPit_value = QLabel("")  # Create a label for the indicato
        self.ui.layout_indicator13 = QHBoxLayout(self.ui.tag5_pitch)  # Assuming you have a placeholder in the .ui file
        self.ui.layout_indicator13.addWidget(self.indicator_apPit)
        self.ui.layout_indicator13.addWidget(self.indicator_apPit_value)
        # Joint angles Inverse Kinematics
        self.indicator_apYa = QLabel("Yaw:")  # Create a label for the indicator
        self.indicator_apYa_value = QLabel("")  # Create a label for the indicato
        self.ui.layout_indicator14 = QHBoxLayout(self.ui.tag5_yaw)  # Assuming you have a placeholder in the .ui file
        self.ui.layout_indicator14.addWidget(self.indicator_apYa)
        self.ui.layout_indicator14.addWidget(self.indicator_apYa_value) 
        
        # End Effector pose
        self.indicator_eeX = QLabel("Pos X:")  # Create a label for the indicator
        self.indicator_eeX_value = QLabel("")  # Create a label for the indicato
        self.ui.layout_indicator15 = QHBoxLayout(self.ui.EF_x)  # Assuming you have a placeholder in the .ui file
        self.ui.layout_indicator15.addWidget(self.indicator_eeX)
        self.ui.layout_indicator15.addWidget(self.indicator_eeX_value)  
        # Joint angles Inverse Kinematics
        self.indicator_eeY = QLabel("Pos Y:")  # Create a label for the indicator
        self.indicator_eeY_value = QLabel("")  # Create a label for the indicato
        self.ui.layout_indicator16 = QHBoxLayout(self.ui.EF_y)  # Assuming you have a placeholder in the .ui file
        self.ui.layout_indicator16.addWidget(self.indicator_eeY)
        self.ui.layout_indicator16.addWidget(self.indicator_eeY_value)  
        # Joint angles Inverse Kinematics
        self.indicator_eeZ = QLabel("Pos Z:")  # Create a label for the indicator
        self.indicator_eeZ_value = QLabel("")  # Create a label for the indicato
        self.ui.layout_indicator17 = QHBoxLayout(self.ui.EF_z)  # Assuming you have a placeholder in the .ui file
        self.ui.layout_indicator17.addWidget(self.indicator_eeZ)
        self.ui.layout_indicator17.addWidget(self.indicator_eeZ_value) 
        # Joint angles Inverse Kinematics
        self.indicator_eeROl = QLabel("Roll:")  # Create a label for the indicator
        self.indicator_eeROl_value = QLabel("")  # Create a label for the indicato
        self.ui.layout_indicator18 = QHBoxLayout(self.ui.EF_roll)  # Assuming you have a placeholder in the .ui file
        self.ui.layout_indicator18.addWidget(self.indicator_eeROl)
        self.ui.layout_indicator18.addWidget(self.indicator_eeROl_value)
        # Joint angles Inverse Kinematics
        self.indicator_eePit = QLabel("Pitch:")  # Create a label for the indicator
        self.indicator_eePit_value = QLabel("")  # Create a label for the indicato
        self.ui.layout_indicator19 = QHBoxLayout(self.ui.EF_pitch)  # Assuming you have a placeholder in the .ui file
        self.ui.layout_indicator19.addWidget(self.indicator_eePit)
        self.ui.layout_indicator19.addWidget(self.indicator_eePit_value)
        # Joint angles Inverse Kinematics
        self.indicator_eeYa = QLabel("Yaw:")  # Create a label for the indicator
        self.indicator_eeYa_value = QLabel("")  # Create a label for the indicato
        self.ui.layout_indicator20 = QHBoxLayout(self.ui.EF_yaw)  # Assuming you have a placeholder in the .ui file
        self.ui.layout_indicator20.addWidget(self.indicator_eeYa)
        self.ui.layout_indicator20.addWidget(self.indicator_eeYa_value) 
         # Embed the Matplotlib plot in the controlQuad page (index 1 of the QStackedWidget)
       
        
        # Connect comboBox to switch pages in the stacked widget
        self.ui.changeRobot.currentIndexChanged.connect(self.change_robot)
        self.ui.jointArm.currentIndexChanged.connect(self.change_jointArm)
        self.ui.jointLeg.currentIndexChanged.connect(self.change_jointLeg)
        self.ui.changeLeg.currentIndexChanged.connect(self.change_Leg)
      

        self.ui.nextPage.clicked.connect(self.nextPage)
        self.ui.nextPage2.clicked.connect(self.nextPage)
        self.ui.prevButtom2.clicked.connect(self.prevPage)
        self.ui.nextPage2_2.clicked.connect(self.nextPage)
        self.ui.prevButtom2_2.clicked.connect(self.prevPage)
        self.ui.nextPage3.clicked.connect(self.nextPage)
        self.ui.prevButtom3.clicked.connect(self.prevPage)
        self.ui.setEF.clicked.connect(self.toggle_EF)
        self.ui.setEF_2.clicked.connect(self.restart)
        self.currentIdx = 0
        # Connect buttons to corresponding functions
        self.ui.setCero.clicked.connect(self.on_set_zero_click)
        self.ui.setMinus10.clicked.connect(self.on_minus_10_click)
        self.ui.setPLus10.clicked.connect(self.on_plus_10_click)
        self.ui.setMax.clicked.connect(self.on_set_max_click)
        
        self.ui.startGait.clicked.connect(self.on_start_gait_click)
        self.ui.stopGait.clicked.connect(self.on_stop_gait_click)
        
        self.ui.startGait_2.clicked.connect(self.on_start_ik_click)
        self.ui.stopGait_2.clicked.connect(self.on_stop_ik_click)  
        
        self.ui.startGait_3.clicked.connect(self.on_start_ik2_click)  
        self.ui.startGait_4.clicked.connect(self.on_stop_origin_click)       
        
        
        self.armJoint = Joint_0
        self.leg = FL
        self.legJoint = Joint_0
        self.Robot = ARM
        self.ui.jointLeg.setVisible(False)
        self.ui.changeLeg.setVisible(False)
        self.ui.jointArm.setVisible(True)

        

        self.ros_node = ros_node
        # Reference to the ROS2 node for interacting with ROS topics
        self.robotMoving = self.ros_node.robotMoving
        self.armMoving = self.ros_node.armMoving
        self.RobotReady = self.ros_node.RobotReady
        # Create a canvas for each plot (x, y, z, roll, pitch, yaw)
        self.plot_canvas = pg.PlotWidget(title="Foots pose URDF")
        self.plot_canvas.showGrid(x=True, y=True)
        #self.plot_canvas.setXRange(-1, 1)  # Ajusta según tu rango real
        #self.plot_canvas.setYRange(-1, 1)
        self.plot_canvas_2 =        pg.PlotWidget(title="Foots Kinematics")
        self.plot_canvas_2.showGrid(x=True, y=True)
        #self.plot_canvas_2.setXRange(-1, 1)  # Ajusta según tu rango real
        #self.plot_canvas_2.setYRange(-1, 1)
        self.plot_canvas_arm =      pg.PlotWidget(title="XY pose URDF")
        self.plot_canvas_arm.showGrid(x=True, y=True)
        #self.plot_canvas_arm.setXRange(-1, 1)
        #self.plot_canvas_arm.setYRange(-1, 1)
        self.plot_canvas_arm_2 =    pg.PlotWidget(title="YZ pose URDF")
        self.plot_canvas_arm_2.showGrid(x=True, y=True)
        #self.plot_canvas_arm_2.setXRange(-1, 1)
        #self.plot_canvas_arm_2.setYRange(-1, 1)
        self.plot_canvas_arm_3 =    pg.PlotWidget(title="XZ pose URDF")
        self.plot_canvas_arm_3.showGrid(x=True, y=True)
        #self.plot_canvas_arm_3.setXRange(-1, 1)
        #self.plot_canvas_arm_3.setYRange(-1, 1)
        # 1) Para plot_canvas (pies URDF):
        self.scatter_feet_urdf = pg.ScatterPlotItem(size=10, brush=pg.mkBrush('b'))
        self.scatter_cog_urdf  = pg.ScatterPlotItem(size=12, brush=pg.mkBrush('r'))
        self.plot_canvas.addItem(self.scatter_feet_urdf)
        self.plot_canvas.addItem(self.scatter_cog_urdf)

        # 2) Para plot_canvas_2 (pies cinemática):
        self.scatter_feet_kin = pg.ScatterPlotItem(size=10, brush=pg.mkBrush('b'))
        self.scatter_cog_kin  = pg.ScatterPlotItem(size=12, brush=pg.mkBrush('r'))
        self.plot_canvas_2.addItem(self.scatter_feet_kin)
        self.plot_canvas_2.addItem(self.scatter_cog_kin)

        # 3) Para plot_canvas_arm (end‐effector XY):
        self.scatter_ee_xy = pg.ScatterPlotItem(size=12, brush=pg.mkBrush('g'))
        self.plot_canvas_arm.addItem(self.scatter_ee_xy)

        # 4) Para plot_canvas_arm_2 (end‐effector YZ):
        self.scatter_ee_yz = pg.ScatterPlotItem(size=12, brush=pg.mkBrush('g'))
        self.plot_canvas_arm_2.addItem(self.scatter_ee_yz)

        # 5) Para plot_canvas_arm_3 (end‐effector XZ):
        self.scatter_ee_xz = pg.ScatterPlotItem(size=12, brush=pg.mkBrush('g'))
        self.plot_canvas_arm_3.addItem(self.scatter_ee_xz)

        # Find the layout of the plotFoot widget (the placeholder in your UI on controlQuad)
        layout = QVBoxLayout(self.ui.plotFoot)  # plotFoot is in the controlQuad page
        layout.addWidget(self.plot_canvas)      # Add the matplotlib canvas to the layout
        layout_2 = QVBoxLayout(self.ui.plotFoot_2)  # plotFoot is in the controlQuad page
        layout_2.addWidget(self.plot_canvas_2)      # Add the matplotlib canvas to the layout
        layout_2 = QVBoxLayout(self.ui.plotArm)  # plotFoot is in the controlQuad page
        layout_2.addWidget(self.plot_canvas_arm)      # Add the matplotlib canvas to the layout
        layout_2 = QVBoxLayout(self.ui.plotArm_2)  # plotFoot is in the controlQuad page
        layout_2.addWidget(self.plot_canvas_arm_2)      # Add the matplotlib canvas to the layout
        layout_2 = QVBoxLayout(self.ui.plotArm_3)  # plotFoot is in the controlQuad page
        layout_2.addWidget(self.plot_canvas_arm_3)      # Add the matplotlib canvas to the layout


        # Create a canvas for each plot (x, y, z, roll, pitch, yaw)
        #self.canvas_x = MyMplCanvas(self, width=5, height=4, dpi=100)
        #self.canvas_y = MyMplCanvas(self, width=5, height=4, dpi=100)
        #self.canvas_z = MyMplCanvas(self, width=5, height=4, dpi=100)
        #self.canvas_roll = MyMplCanvas(self, width=5, height=4, dpi=100)
        #self.canvas_pitch = MyMplCanvas(self, width=5, height=4, dpi=100)
        #self.canvas_yaw = MyMplCanvas(self, width=5, height=4, dpi=100)
        self.max_len = 500  # o 200, dependiendo de cuánto quieras ver “en pantalla”
        self.canvas_x = pg.PlotWidget(title="Position X vs Time")
        self.canvas_x.setBackground('w')   # blanco puro. Si prefieres gris muy suave, usa '#f5f5f5'

        # 2) Fuente del título y etiquetas de los ejes:
        
        self.canvas_x.setTitle("Position X vs Time", color='#333333', size='10pt')
        self.canvas_x.showGrid(x=True, y=True)
        self.canvas_x.getPlotItem().getAxis('bottom').setLabel(text='Time(s)', color='#333333', **{'font-size': '8pt'})
        self.canvas_x.getPlotItem().getAxis('left').setLabel(text='Position(m)', color='#333333', **{'font-size': '8pt'})
        self.canvas_x.getPlotItem().getViewBox().enableAutoRange(False, False)
        self.canvas_x.setYRange(-2, 15, padding=0)
        axis_pen = pg.mkPen(color='#666666', width=1)
        ax_bottom = self.canvas_x.getPlotItem().getAxis('bottom')
        ax_left   = self.canvas_x.getPlotItem().getAxis('left')
        ax_bottom.setPen(axis_pen)
        ax_left.setPen(axis_pen)
        axis_x_y = self.canvas_x.getPlotItem().getAxis('left')
        axis_x_y.setRange(-2, 15)
        axis_x_y.setTickSpacing(5, 1)
        
        self.x_lines = self.canvas_x.plot(
            np.linspace(0, self.max_len-1, self.max_len), 
            np.zeros(self.max_len), 
            pen=pg.mkPen('r', width=2)
        )
        self.canvas_y = pg.PlotWidget(title="Position Y vs Time", color='#333333', size='10pt') 
        self.canvas_y.setBackground('w')
        self.canvas_y.getPlotItem().getAxis('bottom').setLabel(text='Time(s)', color='#333333', **{'font-size': '8pt'})
        self.canvas_y.getPlotItem().getAxis('left').setLabel(text='Position(m)', color='#333333', **{'font-size': '8pt'})
        self.canvas_y.showGrid(x=True, y=True)
        self.canvas_y.getPlotItem().getViewBox().enableAutoRange(False, False)
        self.canvas_y.setYRange(-2, 15, padding=0)
        axis_y_z = self.canvas_y.getPlotItem().getAxis('left')
        axis_y_z.setRange(-2, 15)
        axis_y_z.setTickSpacing(5, 1)
        
        self.y_lines = self.canvas_y.plot(
            np.linspace(0, self.max_len-1, self.max_len), 
            np.zeros(self.max_len), 
            pen=pg.mkPen('r', width=2)
        )
        self.canvas_z = pg.PlotWidget(title="Position Z vs Time", color='#333333', size='10pt')
        self.canvas_z.setBackground('w')
        self.canvas_z.getPlotItem().getAxis('bottom').setLabel(text='Time(s)', color='#333333', **{'font-size': '8pt'})
        self.canvas_z.showGrid(x=True, y=True)
        self.canvas_z.getPlotItem().getViewBox().enableAutoRange(False, False)
        self.canvas_z.setYRange(-2, 15, padding=0)
        axis_x_z = self.canvas_z.getPlotItem().getAxis('left')
        axis_x_z.setRange(-2, 15)
        axis_x_z.setTickSpacing(5, 1)
        
        self.z_lines = self.canvas_z.plot(
            np.linspace(0, self.max_len-1, self.max_len), 
            np.zeros(self.max_len), 
            pen=pg.mkPen('r', width=2)
        )
        self.canvas_roll = pg.PlotWidget(title="Yaw vs Time", color='#333333', size='10pt')
        self.canvas_roll.setBackground('w')
        self.canvas_roll.showGrid(x=True, y=True)
        self.canvas_roll.getPlotItem().getViewBox().enableAutoRange(False, False)
        self.canvas_roll.setYRange(-2, 180, padding=0)
        axis_roll= self.canvas_roll.getPlotItem().getAxis('left')
        axis_roll.setRange(-15, 180)
        axis_roll.setTickSpacing(60, 30)
        self.canvas_roll.getPlotItem().getAxis('bottom').setLabel(text='Time(s)', color='#333333', **{'font-size': '8pt'})
        self.canvas_roll.getPlotItem().getAxis('left').setLabel(text='Position(m)', color='#333333', **{'font-size': '8pt'})
        self.roll_lines = self.canvas_roll.plot(
            np.linspace(0, self.max_len-1, self.max_len), 
            np.zeros(self.max_len), 
            pen=pg.mkPen('r', width=2)
        )
        self.canvas_pitch = pg.PlotWidget(title="Yaw vs Time", color='#333333', size='10pt')
        self.canvas_pitch.setBackground('w')
        self.canvas_pitch.showGrid(x=True, y=True)
        self.canvas_pitch.getPlotItem().getViewBox().enableAutoRange(False, False)
        self.canvas_pitch.setYRange(-2, 180, padding=0)
        axis_pitch= self.canvas_pitch.getPlotItem().getAxis('left')
        axis_pitch.setRange(-15, 180)
        axis_pitch.setTickSpacing(60, 30)
        self.canvas_pitch.getPlotItem().getAxis('bottom').setLabel(text='Time(s)', color='#333333', **{'font-size': '8pt'})
        self.canvas_pitch.getPlotItem().getAxis('left').setLabel(text='Position(m)', color='#333333', **{'font-size': '8pt'})
        self.pitch_lines = self.canvas_pitch.plot(
            np.linspace(0, self.max_len-1, self.max_len), 
            np.zeros(self.max_len), 
            pen=pg.mkPen('r', width=2)
        )
        self.canvas_yaw = pg.PlotWidget(title="Yaw vs Time", color='#333333', size='10pt')
        self.canvas_yaw.setBackground('w')
        self.canvas_yaw.showGrid(x=True, y=True)
        self.canvas_yaw.getPlotItem().getViewBox().enableAutoRange(False, False)
        self.canvas_yaw.setYRange(-2, 180, padding=0)
        axis_yaw= self.canvas_yaw.getPlotItem().getAxis('left')
        axis_yaw.setRange(-15, 180)
        axis_yaw.setTickSpacing(60, 30)
        self.canvas_yaw.getPlotItem().getAxis('bottom').setLabel(text='Time(s)', color='#333333', **{'font-size': '8pt'})
        self.canvas_yaw.getPlotItem().getAxis('left').setLabel(text='Position(m)', color='#333333', **{'font-size': '8pt'})
        self.yaw_lines = self.canvas_yaw.plot(
            np.linspace(0, self.max_len-1, self.max_len), 
            np.zeros(self.max_len), 
            pen=pg.mkPen('r', width=2)
        )
        # Add each canvas to the layout
        layout_acx = QVBoxLayout(self.ui.acc_x)  # plotFoot is in the controlQuad page
        layout_acx.addWidget(self.canvas_x)
        layout_acy = QVBoxLayout(self.ui.acc_y)  # plotFoot is in the controlQuad page
        layout_acy.addWidget(self.canvas_y)
        layout_acz = QVBoxLayout(self.ui.acc_z)  # plotFoot is in the controlQuad page
        layout_acz.addWidget(self.canvas_z)
        layout_roll = QVBoxLayout(self.ui.rot_x)  # plotFoot is in the controlQuad page
        layout_roll.addWidget(self.canvas_roll)
        layout_pitch = QVBoxLayout(self.ui.rot_y)  # plotFoot is in the controlQuad page
        layout_pitch.addWidget(self.canvas_pitch)
        layout_yaw = QVBoxLayout(self.ui.rot_z)  # plotFoot is in the controlQuad page
        layout_yaw.addWidget(self.canvas_yaw)

        

        self.ui.label_2.setText("Static Gait")
        self.ui.label_2.setStyleSheet("font-size: 20px; font-weight: bold; color: blue;")  # Set font size, weight, and color
        # Parámetros para los buffers y ejes
        self.ui.label_6.setText("COG Location")
        self.ui.label_6.setStyleSheet("font-size: 20px; font-weight: bold; color: blue;")  # Set font size, weight, and color
        self.ui.label_7.setText("Inverse Kinematics")
        self.ui.label_7.setStyleSheet("font-size: 20px; font-weight: bold; color: blue;")  # Set font size, weight, and color
        self.ui.label_8.setText("Joints Solution")
        self.ui.label_8.setStyleSheet("font-size: 20px; font-weight: bold; color: blue;")  # Set font size, weight, and color
        self.ui.label_9.setText("AprilTag Location")
        self.ui.label_9.setStyleSheet("font-size: 20px; font-weight: bold; color: blue;")  # Set font size, weight, and color
        self.ui.label_10.setText("End Effector Location")
        self.ui.label_10.setStyleSheet("font-size: 20px; font-weight: bold; color: blue;")  # Set font size, weight, and color
        
        self.ylim_min = -2.0
        self.ylim_max =  2.0
        # Initialize three lines for each plot (x, y, z, roll, pitch, yaw)
        #self.x_lines =      self.canvas_x.axes.plot(np.arange(self.max_len), np.zeros(self.max_len))[0]
        #self.y_lines =      self.canvas_y.axes.plot(np.arange(self.max_len), np.zeros(self.max_len))[0]
        #self.z_lines =      self.canvas_z.axes.plot(np.arange(self.max_len), np.zeros(self.max_len))[0]
        #self.roll_lines =   self.canvas_roll.axes.plot(np.arange(self.max_len), np.zeros(self.max_len))[0]
        #self.pitch_lines =  self.canvas_pitch.axes.plot(np.arange(self.max_len), np.zeros(self.max_len))[0]
        #self.yaw_lines =    self.canvas_yaw.axes.plot(np.arange(self.max_len), np.zeros(self.max_len))[0]
        
        
        self.setAxisPlots()
      
        # QTimer to periodically update the plot
        self.plot_timer = QTimer(self)
        self.plot_timer.timeout.connect(self.plotFoots)
        self.plot_timer.start(1000)  # Refresh plot every 100ms
        
        # QTimer to periodically update the plot
        self.plot_COG = QTimer(self)
        self.plot_COG.timeout.connect(self.plotCog)
        self.plot_COG.start(1000)  # Refresh plot every 100ms

        self.update_joint_positions()
        # Update the indicator based on conditions
        self.update_indicator()
        
    def restart(self):
        self.ros_node.msg_move_robot.m0 = 0.0
        self.ros_node.msg_move_robot.m1 = 0.0
        self.ros_node.msg_move_robot.m2 = 0.0
        self.ros_node.msg_move_robot.m3 = 0.0
        self.ros_node.msg_move_robot.m4 = 0.0
        self.ros_node.msg_move_robot.m5 = 0.0
        self.ros_node.arm_joints[0]
        self.ros_node.arm_joints[1]
        self.ros_node.arm_joints[2]
        self.ros_node.arm_joints[3]
        self.ros_node.arm_joints[4]
        self.ros_node.get_logger().info(f"Updated Arm Joint {Joint_5} Angle: {self.ros_node.msg_move_robot.m5}")
        self.update_joint_positions()
        self.publishMessage()
    def toggle_EF(self):
        current_angle = self.ros_node.msg_move_robot.m5
        if current_angle == 0:
            self.ros_node.msg_move_robot.m5 = 3.14
            new_angle = 3.14
        else:
            self.ros_node.msg_move_robot.m5 = 0.0
            new_angle = 0
        self.ros_node.msg_move_robot.command = "ARM"
        self.ros_node.get_logger().info(f"Updated Arm Joint {Joint_5} Angle: {self.ros_node.msg_move_robot.m5}")
        self.update_joint_positions()
        self.publishMessage()
        
    def update_indicator(self):
        """Update the indicator color or visibility based on some condition.
        self.ros_node.get_logger().info(f"Robot moving: {self.robotMoving}")
        self.ros_node.get_logger().info(f"Arm moving: {self.armMoving}")
        self.ros_node.get_logger().info(f"Robot ready: {self.RobotReady}") """
        self.robotMoving = self.ros_node.robotMoving
        self.armMoving = self.ros_node.armMoving
        self.RobotReady = self.ros_node.RobotReady
        if self.RobotReady:
            self.indicator_robot.setColor("green")
            self.indicator_robot_2.setColor("green")
            self.indicator_robot_3.setColor("green")
            
        else:
            self.indicator_robot.setColor("red")
            self.indicator_robot_2.setColor("red")
            self.indicator_robot_3.setColor("red")
            
        if self.armMoving:
            self.indicator_arm.setColor("green")
            self.indicator_arm_2.setColor("green")
            self.indicator_arm_3.setColor("green")
        else:
            self.indicator_arm.setColor("red")
            self.indicator_arm_2.setColor("red")
            self.indicator_arm_3.setColor("red")
            
        if self.robotMoving:
            self.indicator_leg.setColor("green")
            self.indicator_leg_2.setColor("green")
            self.indicator_leg_3.setColor("green")
        else:
            self.indicator_leg.setColor("red")
            self.indicator_leg_2.setColor("red")
            self.indicator_leg_3.setColor("red")
                
        self.indicator_robot.setAutoFillBackground(True)
        self.indicator_robot_2.setAutoFillBackground(True)
        self.indicator_robot_3.setAutoFillBackground(True)
        self.indicator_robot.repaint()
        self.indicator_robot_2.repaint()
        self.indicator_robot_3.repaint()
        self.indicator_robot.update()
        self.indicator_robot_2.update()
        self.indicator_robot_3.update()
        
        
        self.indicator_arm.repaint()
        self.indicator_arm_2.repaint()
        self.indicator_arm_3.repaint()
        self.indicator_arm.update()
        self.indicator_arm_2.update()
        self.indicator_arm_3.update()
        
        self.indicator_leg.repaint()
        self.indicator_leg_2.repaint()
        self.indicator_leg_3.repaint()
        
        self.indicator_leg.update()
        self.indicator_leg_2.update()
        self.indicator_leg_3.update()
    def plotCog(self):
        try:
            self.update_plots()
        finally:
            pass
        
    def plotFoots(self):
        self.update_indicator()

        footPose = self.ros_node.footPose        # lista de 4 pares [x,y]
        footPose_2 = self.ros_node.footPose_k    # lista de 4 pares [x,y]
        cogPose = self.ros_node.cogPose          # [x, y, z]
        ee     = self.ros_node.endEfector        # [x, y, z, rot...]
        aprilTagPose = self.ros_node.aprilTagPose  # [x, y, z, roll, pitch, yaw]
        ik = self.ros_node.ikSol  
        self.indicator_j0_value.setText(f"{ik[0]:.2f},  "f"{np.rad2deg(ik[0]):.2f}")
        self.indicator_j1_value.setText(f"{ik[1]:.2f},  "f"{np.rad2deg(ik[1]):.2f}")
        self.indicator_j2_value.setText(f"{ik[2]:.2f},  "f"{np.rad2deg(ik[2]):.2f}")
        self.indicator_j3_value.setText(f"{ik[3]:.2f},  "f"{np.rad2deg(ik[3]):.2f}")
        self.indicator_j4_value.setText(f"{ik[4]:.2f},  "f"{np.rad2deg(ik[4]):.2f}")
        # Actualizar indicadores de posición de las articulaciones:
        self.indicator_apX_value.setText(f"{aprilTagPose[0]:.2f}")
        self.indicator_apY_value.setText(f"{aprilTagPose[1]:.2f}")
        self.indicator_apZ_value.setText(f"{aprilTagPose[2]:.2f}")
        self.indicator_apROl_value.setText(f"{aprilTagPose[3]:.2f}")
        self.indicator_apPit_value.setText(f"{aprilTagPose[4]:.2f}")
        self.indicator_apYa_value.setText(f"{aprilTagPose[5]:.2f}")
        self.indicator_eeX_value.setText(f"{ee[0]:.2f}")
        self.indicator_eeY_value.setText(f"{ee[1]:.2f}")
        self.indicator_eeZ_value.setText(f"{ee[2]:.2f}")
        self.indicator_eeROl_value.setText(f"{ee[3]:.2f}")
        self.indicator_eePit_value.setText(f"{ee[4]:.2f}")
        self.indicator_eeYa_value.setText(f"{ee[5]:.2f}")
        
        
        
        # 1) Actualizar pies URDF + COG en plot_canvas:
        xs_urdf = [p[0] for p in footPose]
        ys_urdf = [p[1] for p in footPose]
        self.scatter_feet_urdf.setData(x=xs_urdf, y=ys_urdf)
        # COG proyectado en XY:
        self.scatter_cog_urdf.setData(x=[cogPose[0]], y=[cogPose[1]])

        # 2) Actualizar pies cinemática + COG en plot_canvas_2:
        xs_kin = [p[0] for p in footPose_2]
        ys_kin = [p[1] for p in footPose_2]
        self.scatter_feet_kin.setData(x=xs_kin, y=ys_kin)
        self.scatter_cog_kin.setData(x=[cogPose[0]], y=[cogPose[1]])

        # 3) End‐effector XY en plot_canvas_arm:
        self.scatter_ee_xy.setData(x=[ee[0]], y=[ee[1]])

        # 4) End‐effector YZ en plot_canvas_arm_2:
        self.scatter_ee_yz.setData(x=[ee[1]], y=[ee[2]])

        # 5) End‐effector XZ en plot_canvas_arm_3:
        self.scatter_ee_xz.setData(x=[ee[0]], y=[ee[2]])
       
    def nextPage(self):
        if self.currentIdx < 3:
            self.currentIdx +=1
            self.stacked_widget.setCurrentIndex(self.currentIdx)
    def prevPage(self):
        if self.currentIdx > 0:
            self.currentIdx -=1
            self.stacked_widget.setCurrentIndex(self.currentIdx)


    def change_robot(self, index):
        """
        Changes the page in the QStackedWidget based on the comboBox selection.
        """
        print(f"Changing to page {index}")
        if index == 0:
            self.Robot = ARM
            self.ui.jointLeg.setVisible(False)
            self.ui.changeLeg.setVisible(False)
            self.ui.jointArm.setVisible(True)
        elif index == 1:
            self.Robot = QUAD
            self.ui.jointLeg.setVisible(True)
            self.ui.changeLeg.setVisible(True)
            self.ui.jointArm.setVisible(False)
    def change_jointArm(self, index):
        """
        Change joint to move
        """
        
        self.ros_node.get_logger().info(f"Changing joint arm  {index}")
        self.armJoint = index
       


    def change_jointLeg(self, index):
        """
        Change joint to move
        """
        self.ros_node.get_logger().info(f"Changing joint leg  {index}")
        self.legJoint = index
        
    def change_Leg(self, index):
        """
        Change joint to move
        """
        self.ros_node.get_logger().info(f"Changing leg {index}")
        self.leg = index
        
    def on_set_zero_click(self):
        """
        This function is called when the '0º' button is clicked. It prints a message to the log.
        """
        angle = 0
        self.moveRobot(angle,True)
        print("Set 0º clicked")
        
    def on_minus_10_click(self):
        """
        This function is called when the '-10º' button is clicked. It prints a message to the log.
        """
        print("Set -10º clicked")
        angle = -0.3142
        self.moveRobot(angle,False)
        

    def on_plus_10_click(self):
        """
        This function is called when the '+10º' button is clicked. It prints a message to the log.
        """
        print("Set +10º clicked")
        angle = 0.3142
        self.moveRobot(angle,False)
        
        

    def on_set_max_click(self):
        """
        This function is called when the '180º' button is clicked. It prints a message to the log.
        """
        print("Set 180º clicked")
        angle = 3.142
        self.moveRobot(angle,True)  
        
    def on_start_gait_click(self):
        print("Set 180º clicked")
        angle = 3.142
        
        self.ros_node.msg_move.robot = 'm4'
        self.ros_node.msg_move_robot.command = 'm4'
        
        self.publishMessage()
    def on_stop_ik_click(self):
        if self.ros_node.getIk:
            self.ros_node.msg_ik.data = False
            self.ros_node.publisher_getIkSolution.publish(self.ros_node.msg_ik)
            self.ros_node.getIk = False
        else:
            self.ros_node.msg_ik.data = True
            self.ros_node.getIk
            self.ros_node.publisher_getIkSolution.publish(self.ros_node.msg_ik)
            
    def on_start_ik_click(self):
        print("Set 180º clicked")
        
        self.ros_node.arm_joints[Joint_0] = self.ros_node.ikSol[0]
        self.ros_node.arm_joints[Joint_1] = self.ros_node.ikSol[1]
        self.ros_node.arm_joints[Joint_2] = self.ros_node.ikSol[2]
        self.ros_node.arm_joints[Joint_3] = self.ros_node.ikSol[3]
        self.ros_node.arm_joints[Joint_4] = self.ros_node.ikSol[4]
        
        self.ros_node.msg_move.robot = "ARM"
        self.ros_node.msg_move_robot.command = "ARM"
        self.ros_node.msg_move.leg = "nn"
        self.ros_node.msg_move_robot.m5= 1.0  # Reset end-effector joint to 0
        self.update_joint_positions()
        self.publishMessage()
    def on_start_ik2_click(self):
        print("Set 180º clicked")
        
        self.ros_node.arm_joints[Joint_0] = self.ros_node.ikSol[0]
        self.ros_node.arm_joints[Joint_1] = self.ros_node.ikSol[1]
        self.ros_node.arm_joints[Joint_2] = self.ros_node.ikSol[2]
        self.ros_node.arm_joints[Joint_3] = self.ros_node.ikSol[3]
        self.ros_node.arm_joints[Joint_4] = self.ros_node.ikSol[4]
        
        self.ros_node.msg_move.robot = "ARM"
        self.ros_node.msg_move_robot.command = "ARM"
        self.ros_node.msg_move.leg = "nn"
        self.ros_node.msg_move_robot.m5=  0.0 # Reset end-effector joint to 0
        self.update_joint_positions()
        self.publishMessage()
        
    def on_stop_origin_click(self):
        self.ros_node.msg_move.robot = 'origin'
        self.ros_node.msg_move_robot.command = 'origin'
  
        self.publishMessage()
        
    def on_stop_gait_click(self):
        
        self.ros_node.msg_move.robot = 'origin'
        self.ros_node.msg_move_robot.command = 'origin'
        msg = Bool()
        
        self.publishMessage()
    def moveRobot(self,angle,Force):
        currentAngle = 0
        
        
        if self.Robot == ARM:
            self.ros_node.msg_move.robot = "ARM"
            self.ros_node.msg_move_robot.command = "ARM"
            self.ros_node.msg_move.leg = "nn"
            self.armMoving = True
            
            current_angle = self.ros_node.arm_joints[self.armJoint]
        
            # Log current joint angle for debugging
            self.ros_node.get_logger().info(f"Current Arm Joint {self.armJoint} Angle: {current_angle}")

            # Calculate new joint angle
            if not Force:
                if angle + current_angle <= 0:
                    new_angle = 0
                elif angle + current_angle >= 3.142:
                    new_angle = 3.142
                else:
                    new_angle = angle + current_angle
            else:
                new_angle = angle
            if self.armJoint == Joint_0 and ( angle > 0 and new_angle < 0.827):
                new_angle = 0.827
            elif self.armJoint == Joint_0 and (angle <= 0.827 and new_angle < 0.827 ):
                new_angle = 0
            

            # Set updated joint angle
            
            self.ros_node.arm_joints[self.armJoint] = new_angle

            # Log updated joint angle for debugging
            updated_angle = new_angle
            self.ros_node.get_logger().info(f"Updated Arm Joint {self.armJoint} Angle: {updated_angle}")

            self.ros_node.msg_move.joint = str(self.armJoint)
            self.ros_node.msg_move.angle =  float(new_angle )
        elif self.Robot == QUAD:
            
            self.ros_node.msg_move.robot = 'QUAD'
           # self.ros_node.msg_move_robot = 'QUAD'
            
            if self.leg == FL:
                
                # self.ros_node.get_logger().info(f"FL currentAngle  {currentAngle}")
                current_angle = self.ros_node.leg_joints_FL[self.legJoint]
                     # Calculate new joint angle
                if not Force:
                    if angle + current_angle <= 0:
                        new_angle = 0
                    elif angle + current_angle >= 3.142:
                        new_angle = 3.142
                    else:
                        new_angle = angle + current_angle
                else:
                    new_angle = angle
                
                self.ros_node.leg_joints_FL[self.legJoint] = new_angle
                self.ros_node.msg_move.leg = str("FL")
                self.ros_node.msg_move.joint = str(self.legJoint)
                self.ros_node.msg_move.angle =  float(new_angle)
            elif self.leg == FR:
                 # Calculate new joint angle
                current_angle = self.ros_node.leg_joints_FR[self.legJoint]
                if not Force:
                    if angle + current_angle <= 0:
                        new_angle = 0
                    elif angle + current_angle >= 3.142:
                        new_angle = 3.142
                    else:
                        new_angle = angle + current_angle
                else:
                    new_angle = angle 
                
                  
                self.ros_node.leg_joints_FR[self.legJoint] = new_angle
                self.ros_node.msg_move.leg =str("FR")
                self.ros_node.msg_move.joint = str(self.legJoint)
                self.ros_node.msg_move.angle =  float(new_angle)
            elif self.leg == BR:
                current_angle = self.ros_node.leg_joints_BR[self.legJoint] 
                if not Force:
                    if angle + current_angle <= 0:
                        new_angle = 0
                    elif angle + current_angle >= 3.142:
                        new_angle = 3.142
                    else:
                        new_angle = angle + current_angle
                else:
                    new_angle = angle 
                
                self.ros_node.leg_joints_BR[self.legJoint] = new_angle
                self.ros_node.msg_move.leg =str("FR")
                self.ros_node.msg_move.joint = str(self.legJoint)
                self.ros_node.msg_move.angle =  float(new_angle)
            elif self.leg == BL:
               
                current_angle = self.ros_node.leg_joints_BL[self.legJoint]
                if not Force:
                    if angle + current_angle <= 0:
                        new_angle = 0
                    elif angle + current_angle >= 3.142:
                        new_angle = 3.142
                    else:
                        new_angle = angle + current_angle
                else:
                    new_angle = angle 
                
                self.ros_node.leg_joints_BL[self.legJoint] = new_angle
                self.ros_node.msg_move.leg =str("FR")
                self.ros_node.msg_move.joint = str(self.legJoint)
                self.ros_node.msg_move.angle =  float(new_angle)

        self.update_joint_positions()
        self.publishMessage()
    def update_joint_positions(self):
        # Update the arm joint positions
        position = [
            

            # Leg FL (Front Left)
            
            float(self.ros_node.leg_joints_FL[Joint_0]),  # Update 'joint_0' of the FL leg
            float(self.ros_node.leg_joints_FL[Joint_1]),  # Update 'joint_0' of the FL leg
            float(self.ros_node.leg_joints_FL[Joint_2]),  # Update 'joint_0' of the FL leg
            
            # Leg FR (Front Right)
            
            float(self.ros_node.leg_joints_FR[Joint_0]),  # Update 'joint_0' of the FL leg
            float(self.ros_node.leg_joints_FR[Joint_1]),  # Update 'joint_0' of the FL leg
            float(self.ros_node.leg_joints_FR[Joint_2]),  # Update 'joint_0' of the FL leg
            
            # Leg BL (Back Right)
            
            float(self.ros_node.leg_joints_FL[Joint_0]),  # Update 'joint_0' of the FL leg
            float(self.ros_node.leg_joints_FL[Joint_1]),  # Update 'joint_0' of the FL leg
            float(self.ros_node.leg_joints_FL[Joint_2]),  # Update 'joint_0' of the FL leg
            
            # Leg BR (Back Right)
           
            float(self.ros_node.leg_joints_FR[Joint_0]),  # Update 'joint_0' of the FL leg
            float(self.ros_node.leg_joints_FR[Joint_1]),  # Update 'joint_0' of the FL leg
            float(self.ros_node.leg_joints_FR[Joint_2]),  # Update 'joint_0' of the FL leg

            float(self.ros_node.arm_joints[Joint_0]),  # Update ARM
            float(self.ros_node.arm_joints[Joint_1]),  # Update 'joint_0' of the FL leg
            float(self.ros_node.arm_joints[Joint_2]),  # Update 'joint_0' of the FL leg
            float(self.ros_node.arm_joints[Joint_3]) ,  # Update 'joint_0' of the FL leg
            float(self.ros_node.arm_joints[Joint_4])  # Update 'joint_0' of the FL leg
            
        ]

        # Ensure the JointState message header timestamp is updated

        self.ros_node.joint_state_msg.position = [float(value) for value in  position ]

        self.ros_node.joint_state_msg.header.stamp = self.ros_node.get_clock().now().to_msg()
        
        # Publish the updated joint states
        self.ros_node.joint_states_pub.publish(self.ros_node.joint_state_msg)

    def publishMessage(self):
        self.ros_node.msg_move.angle =  float(self.ros_node.msg_move.angle )*180/np.pi
       
        
        self.ros_node.msg_move_robot.m0 =    round(float(self.ros_node.arm_joints[Joint_0])*180/np.pi ,2)
        self.ros_node.msg_move_robot.m1 =    round(float(self.ros_node.arm_joints[Joint_1])*180/np.pi ,2)
        self.ros_node.msg_move_robot.m2 =    round(float(self.ros_node.arm_joints[Joint_2])*180/np.pi ,2)
        self.ros_node.msg_move_robot.m3 =    round(float(self.ros_node.arm_joints[Joint_3])*180/np.pi ,2)
        self.ros_node.msg_move_robot.m4 =    round(float(self.ros_node.arm_joints[Joint_4])*180/np.pi,2)
        #self.ros_node.msg_move_robot.m5 =    round(float(self.ros_node.arm.getJoint(Joint_5))*180/np.pi,2)
        self.ros_node.publisher_commandRobot.publish(self.ros_node.msg_move_robot)

    
    def update_plots(self):
        """Update the plots with new data from ROS."""
        

         # 1) Convertir cada deque a numpy array
        data_x     = np.array(self.ros_node.data_x)
        data_y     = np.array(self.ros_node.data_y)
        data_z     = np.array(self.ros_node.data_z)
        data_roll  = np.array(self.ros_node.data_roll)
        data_pitch = np.array(self.ros_node.data_pitch)
        data_yaw   = np.array(self.ros_node.data_yaw)

        # 2) Si no hay datos, salgo
        if (data_x.size == 0 or data_y.size == 0 or data_z.size == 0
            or data_roll.size == 0 or data_pitch.size == 0 or data_yaw.size == 0):
            return

        # 3) Crear buffers completos de longitud self.max_len (todos ceros)
        arr_x     = np.zeros(self.max_len)
        arr_y     = np.zeros(self.max_len)
        arr_z     = np.zeros(self.max_len)
        arr_roll  = np.zeros(self.max_len)
        arr_pitch = np.zeros(self.max_len)
        arr_yaw   = np.zeros(self.max_len)

        # 4) Copiar las últimas n muestras al final de cada buffer
        n_x     = data_x.size
        n_y     = data_y.size
        n_z     = data_z.size
        n_roll  = data_roll.size
        n_pitch = data_pitch.size
        n_yaw   = data_yaw.size

        arr_x[-n_x:]     = data_x
        arr_y[-n_y:]     = data_y
        arr_z[-n_z:]     = data_z
        arr_roll[-n_roll:]  = data_roll
        arr_pitch[-n_pitch:] = data_pitch
        arr_yaw[-n_yaw:]   = data_yaw

        # 5) Actualizar cada PlotDataItem (llama a setData)
        self.x_lines   .setData(arr_x)
        self.y_lines   .setData(arr_y)
        self.z_lines   .setData(arr_z)
        self.roll_lines.setData(arr_roll)
        self.pitch_lines.setData(arr_pitch)
        self.yaw_lines .setData(arr_yaw)
    
    def setAxisPlots(self):
        
        #self.canvas_x.axes.set_xlabel('Time (s)')
        #self.canvas_x.axes.set_ylabel('Position X (m)')
        #self.canvas_x.axes.set_title('Position X vs Time')
        #self.canvas_x.axes.relim()
        #self.canvas_x.axes.autoscale_view()
        #self.canvas_x.fig.tight_layout()
        #self.canvas_x.fig.subplots_adjust(left=0.2, right=0.8, top=0.8, bottom=0.2)
        #self.canvas_x.axes.grid(True)
        #self.canvas_y.axes.set_xlabel('Time (s)')
        #self.canvas_y.axes.set_ylabel('Position Y (m)')
        #self.canvas_y.axes.set_title('Position Y vs Time')
        #self.canvas_y.fig.tight_layout()
        #self.canvas_y.fig.subplots_adjust(left=0.2, right=0.8, top=0.8, bottom=0.2)
        #self.canvas_y.axes.relim()
        #self.canvas_y.axes.autoscale_view()
        #self.canvas_y.axes.grid(True)
        #self.canvas_z.axes.set_xlabel('Time (s)')
        #self.canvas_z.axes.set_ylabel('Position Z (m)')
        #self.canvas_z.axes.set_title('Position Z vs Time')
        #self.canvas_z.fig.tight_layout()
        #self.canvas_z.fig.subplots_adjust(left=0.2, right=0.8, top=0.8, bottom=0.2)
        #self.canvas_z.axes.relim()
        #self.canvas_z.axes.autoscale_view()
        #self.canvas_z.axes.grid(True)
        #self.canvas_roll.axes.set_xlabel('Time (s)')
        #self.canvas_roll.axes.set_ylabel('Orientation roll (rad)')
        #self.canvas_roll.axes.set_title('Roll vs Time')
        #self.canvas_roll.fig.tight_layout()
        #self.canvas_roll.fig.subplots_adjust(left=0.2, right=0.8, top=0.8, bottom=0.2)
        #self.canvas_roll.axes.relim()
        #self.canvas_roll.axes.autoscale_view()
        #self.canvas_roll.axes.grid(True)
        #self.canvas_pitch.axes.set_xlabel('Time (s)')
        #self.canvas_pitch.axes.set_ylabel('Orientation pitch (rad)')
        #self.canvas_pitch.axes.set_title('Pitchvs Time')
        #self.canvas_pitch.fig.tight_layout()
        #self.canvas_pitch.fig.subplots_adjust(left=0.2, right=0.8, top=0.8, bottom=0.2)
        #self.canvas_pitch.axes.relim()
        #self.canvas_pitch.axes.autoscale_view()
        #self.canvas_pitch.axes.grid(True)
        #self.canvas_yaw.axes.set_xlabel('Time (s)')
        #self.canvas_yaw.axes.set_ylabel('Orientation yaw (rad)')
        #self.canvas_yaw.axes.set_title('Yaw vs Time')
        #self.canvas_yaw.fig.tight_layout()
        #self.canvas_yaw.fig.subplots_adjust(left=0.2, right=0.8, top=0.8, bottom=0.2)
        #self.canvas_yaw.axes.relim()
        #self.canvas_yaw.axes.autoscale_view()
        #self.canvas_yaw.axes.grid(True)
        return
        
def ros_spin_thread(node):
    """
    This function runs the ROS2 spin loop in a separate thread to process ROS2 messages.
    """
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Error in ROS spin thread: {e}")
def main():
    """
    Main function that sets up the ROS2 node, starts the PyQt5 application, and runs the ROS2
    spin loop in a separate thread.
    """
    rclpy.init()  # Initialize ROS2
    app = QApplication(sys.argv)  # Initialize the PyQt application
    # ROS 2 node
    
    ros_node = MyNode()  # Create the ROS2 node with the QLabel passed in
    # Create the PyQt5 window and pass the ROS2 node to it
    window = MyWindow(ros_node)
    window.show()
    window.plotFoots()
    # Run the ROS2 node in a separate thread so that it doesn't block the UI thread
    ros_thread = Thread(target=ros_spin_thread, args=(ros_node,), daemon=True)
    ros_thread.start()
    # Regularly publish joint states every 100ms
    timer = ros_node.create_timer(0.5, ros_node.publish_joint_states)
    # Ensure ROS2 shutdown when PyQt application exits
    app.aboutToQuit.connect(lambda: ros_node.destroy_node())
    app.aboutToQuit.connect(lambda: rclpy.shutdown())
    sys.exit(app.exec_())  # Start the PyQt5 application event loop
if __name__ == '__main__':
    main()