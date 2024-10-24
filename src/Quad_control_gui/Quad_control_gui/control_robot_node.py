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
import numpy as np

import rclpy
from rclpy.node import Node

from Quad_control_gui.MyMplCanvas import MyMplCanvas,IndicatorWidget
from Quad_control_gui.control_robot import Ui_RobotControl
from Quad_control_gui.IkQuad import FiveDOFArm , ForwardKinematicsLeg

from tf2_ros import TransformListener, Buffer

from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from robot_interfaces.msg import Anglemotor
from robot_interfaces.msg import Command
from robot_interfaces.msg import Mpu,COGframe
from sensor_msgs.msg import JointState

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
    X_joint = 'X_joint'
    Y_joint = 'Y_joint'
    Z_joint = 'Z_joint'
    Roll_joint = 'Roll_joint'
    Pitch_joint = 'Pitch_joint'
    Yaw_joint = 'Yaw_joint'
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

L1 = 0.1
L2 = 25 / 100
L3 = 25 / 100 
a = 0.6
b = 0.2


a1 = 0.1
a2 =  0.35
a3 = 0.3
a4 = 0.08
a5 = 0.25

"""
dh_Arm = [(0  ,     -np.pi/2,  a1 ,     0), #m0 a m1
          (a2 ,      0   ,  0  ,        -np.pi/2),  # m1 a m2 Upper arm
          (a3,       0  ,  0 ,         np.pi/2),  # m2 a m3 Lower arm
          (a4 ,      0   ,  0,          0),# m3 a m4 Wrist
          (a5 ,      0   ,  0,          0)]  #m4 a effector
"""  
"""          
dh_Arm = [(0  ,     -np.pi/2,  a1 ,     0), #m0 a m1
          (a2 ,      0   ,  0  ,        -np.pi/2),  # m1 a m2 Upper arm
          (a3+a4,       0  ,  0 ,         np.pi/2),  # m2 a m4 Lower arm
          (a5 ,      np.pi   ,  0,          0)]  #m4 a effector 



dh_Arm = [(0  ,     -np.pi/2,  a1 ,     0), #m0 a m1
          (a2 ,      0   ,  0  ,        -np.pi/2),  # m1 a m2 Upper arm
          (a3,       np.pi/2  , 0   ,   np.pi/2),  # m2 a m3 Lower arm
          (a4,      -np.pi/2  ,        0,   0),  # m3 a m4 Wrist THIS FAILED JOINT3 
          (a5 ,       0 ,  0,          0)]  #m4 a effector 

          """ 
dh_Arm = [          (0          , -np.pi/2      ,   a1      ,       0       ), #m0 a m1
                    (a2         ,  0            ,   0       ,       -np.pi/2),  # m1 a m2 Upper arm
                    (a3+a4+a5   , -np.pi/2      ,   0       ,       np.pi/2 ),  # m2 a m3 Lower arm
                    (0          , np.pi/2       ,   0       ,       0       ),  # m3 a m4 Wrist THIS FAILED JOINT3 
                    (0          , 0             ,   0       ,       0       )]  #m4 a effector 


dh_Right_leg = [  
                    (L1         ,     np.pi/2   ,  0       ,        0        ),  # Hip to knee
                    (L2         ,     0         ,  0       ,        -0.6     ), # Knee to ankle
                    (L3         ,     0         ,  0       ,        1.7      )]  # Ankle to foot


dh_Left_leg = [   
                    (L1         ,     np.pi/2   ,  0        ,       0        ), # Hip to knee
                    (L2         ,     0         ,  0        ,       -0.6     ), # Knee to ankle
                    (L3         ,     0         ,  0        ,       1.7      )] # Ankle to foot    
edge_FL = np.array([a , b])
edge_BL = np.array([-a , b])
edge_FR = np.array([a , -b])
edge_BR = np.array([-a , -b])
class MyNode(Node):
    def __init__(self):
        super().__init__('control_robot_node')
        
        self.data1 = 0
        self.data2 = 0
        self.msg_move = Anglemotor()
        self.publisher = self.create_publisher(Anglemotor, 'motor_angles', 10)
        self.subscription = self.create_subscription(Command, 'command_robot', self.listener_callback, 10)
        self.joint_states_pub = self.create_publisher(JointState,'joint_states',10)
        self.subscription_cog = self.create_subscription(COGframe, 'kalman_cog_frame_3', self.cog_callback, 10)

        # Sample data storage for x, y, z, roll, pitch, yaw with 3 sources for each
        self.data_x = []
        self.data_y = []
        self.data_z = []
        self.data_roll = []
        self.data_pitch = []
        self.data_yaw = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.footPose = [[0,0],[0,0],[0,0],[0,0]]
        self.footPose_k = [[0,0],[0,0],[0,0],[0,0]]
        self.endEfector = [0,0,0,0,0,0]
        self.cogPose = [0,0,0]
        self.cogOR = [0,0,0]
        self.COG = np.append(self.cogPose,self.cogOR)

       
        # Create Arm objests
        self.armJoint = np.array([0.0,0.0,0.0,0.0,0.0])
        self.arm = FiveDOFArm(dh_Arm,self.armJoint ,self.COG )
        self.legJoints_FL = np.array([0.0 , 0.0 , 0.0])
        self.legJoints_FR = copy.deepcopy(self.legJoints_FL)  # Creates an independent list
        self.legJoints_BL = copy.deepcopy(self.legJoints_FL)
        self.legJoints_BR = copy.deepcopy(self.legJoints_FL)
        self.R_Front = ForwardKinematicsLeg(dh_Right_leg[:],self.COG,edge_FR,self.legJoints_FL[:])
        self.R_Back = ForwardKinematicsLeg(dh_Right_leg[:],self.COG,edge_BR,self.legJoints_FR[:])
        self.L_Front = ForwardKinematicsLeg(dh_Left_leg[:],self.COG,edge_FL,self.legJoints_BR[:])
        self.L_Back = ForwardKinematicsLeg(dh_Left_leg[:],self.COG,edge_BL,self.legJoints_BL[:])
        

         # Timer to periodically check the transform
        self.timer_footPose = self.create_timer(0.5, self.timer_foot_callback)
        self.timer_foot_callback()


        self.joint_state_msg = JointState()
         # List of joint names from your URDF
        self.joint_state_msg.name = self.joint_state_msg.name = [joint.value for joint in Joints]
        
        self.joint_state_msg.position = [float(value) for value in [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

        # Set the current time for the header
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the initial joint states
        
        self.get_logger().info('Initial joint states published.')
        self.joint_states_pub.publish(self.joint_state_msg)
        self.RobotReady = False
    def add_data(self, msg, index):
        """Add received data to the corresponding lists."""
        self.data_x.append(msg.pos_x)
        self.data_y.append(msg._pos_y)
        self.data_z.append(msg.pos_z)
        self.data_roll.append(msg.roll)
        self.data_pitch.append(msg.pitch)
        self.data_yaw.append(msg.yaw)

        if len(self.data_x) > 1000:
            self.data_x.pop(0)
        
        if len(self.data_y) > 1000:
            self.data_y.pop(0)
        
        if len(self.data_z) > 1000:
            self.data_z.pop(0)
        
        if len(self.data_roll) > 1000:
            self.data_roll.pop(0)
        
        if len(self.data_pitch) > 1000:
            self.data_pitch.pop(0)
        
        if len(self.data_yaw) > 1000:
            self.data_yaw.pop(0) 
        

        

    def cog_callback(self, msg):
        """Callback for cog_data topic."""
        self.add_data(msg, 0)
    def timer_foot_callback(self):
        foot_joint= ['frontLeft_foot','frontRight_foot','backRight_foot','backLeft_foot'] 
        arm_joint= ['endEfector'] 
        x = 0
        for i in foot_joint:
            
            try:
                
                # Look for the transform from the world frame to the end effector
                now = rclpy.time.Time()
                transform = self.tf_buffer.lookup_transform('odom', i, now)
                
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
        
       
        foot_RB = self.R_Back.FootPose()
        foot_RF = self.R_Front.FootPose()
        foot_LB = self.L_Back.FootPose()
        foot_LF = self.L_Front.FootPose()
        self.footPose_k[0] = foot_LF[0:2]
        self.footPose_k[1] = foot_RF[0:2]
        self.footPose_k[2] = foot_RB[0:2]
        self.footPose_k[3] = foot_LB[0:2]

        
        """
        self.get_logger().info(f"foot_LF,: x={foot_LF[0]}, y={foot_LF[1]}, z={foot_LF[2]}")
        self.get_logger().info(f"foot_RF,: x={foot_RF[0]}, y={foot_RF[1]}, z={foot_RF[2]}")
        self.get_logger().info(f"foot_RB,: x={foot_RB[0]}, y={foot_RB[1]}, z={foot_RB[2]}")
        self.get_logger().info(f"foot_LB,: x={foot_LB[0]}, y={foot_LB[1]}, z={foot_LB[2]}")
        
        """
            # Look for the transform from the world frame to the end effector
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform('odom', 'Yaw_body', now)
            
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
            self.arm.changeArmBase(self.COG)
        except Exception as e:
            pass
            #self.get_logger().warn(f"Could not get transform: {str(e)}")
        try:
                
                # Look for the transform from the world frame to the end effector
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform('odom', 'endEfector', now)
            
            # Extract the translation (position) and rotation (orientation)
            trans = transform.transform.translation
            rot = transform.transform.rotation  
            
            # Print the position and orientation of the end effector
            #self.get_logger().info(f"End Effecor: x={trans.x}, y={trans.y}, z={trans.z}")
                
            self.endEfector = [trans.x ,trans.y,trans.z,rot.x,rot.y,rot.z]
            endPose ,orientation= self.arm.ArmPose()
        
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
        
    def publish_message(self, message):
        """
        Publishes a string message to the 'output_topic'.
        """
        
        self.publisher.publish(self.msg_move)
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
        self.indicator_quad = IndicatorWidget()  # Create the indicator widget
        self.indicator_label = QLabel("Robot Status:")  # Create a label for the indicator

        self.ui.layout_indicator = QHBoxLayout(self.ui.quadMoving)  # Assuming you have a placeholder in the .ui file
        self.ui.layout_indicator.addWidget(self.indicator_label)
        self.ui.layout_indicator.addWidget(self.indicator_quad)  

        self.indicator_arm= IndicatorWidget()  # Create the indicator widget
        self.indicator_label_arm = QLabel("Arm moving:")  # Create a label for the indicator
        self.ui.layout_indicator2 = QHBoxLayout(self.ui.armMoving)  # Assuming you have a placeholder in the .ui file
        self.ui.layout_indicator2.addWidget(self.indicator_label_arm)
        self.ui.layout_indicator2.addWidget(self.indicator_arm)  

        self.indicator_leg = IndicatorWidget()  # Create the indicator widget
        self.indicator_label_leg = QLabel("Quad moving:")  # Create a label for the indicator
        self.ui.layout_indicator3 = QHBoxLayout(self.ui.LegMoving)  # Assuming you have a placeholder in the .ui file
        self.ui.layout_indicator3.addWidget(self.indicator_label_leg)
        self.ui.layout_indicator3.addWidget(self.indicator_leg)  
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
        self.currentIdx = 0
        # Connect buttons to corresponding functions
        self.ui.setCero.clicked.connect(self.on_set_zero_click)
        self.ui.setMinus10.clicked.connect(self.on_minus_10_click)
        self.ui.setPLus10.clicked.connect(self.on_plus_10_click)
        self.ui.setMax.clicked.connect(self.on_set_max_click)
                
        # Reference to the ROS2 node for interacting with ROS topics
        self.robotMoving = False
        
        self.armJoint = Joint_0
        self.leg = FL
        self.legJoint = Joint_0
        self.Robot = ARM
        self.ui.jointLeg.setVisible(False)
        self.ui.changeLeg.setVisible(False)
        self.ui.jointArm.setVisible(True)

        

        self.ros_node = ros_node

        # Create a canvas for each plot (x, y, z, roll, pitch, yaw)
        self.plot_canvas = MyMplCanvas(self, width=4, height=4, dpi=100)
        self.plot_canvas_2 = MyMplCanvas(self, width=4, height=4, dpi=100)
        self.plot_canvas_arm = MyMplCanvas(self, width=4, height=4, dpi=100)
        self.plot_canvas_arm_2 = MyMplCanvas(self, width=4, height=4, dpi=100)
        self.plot_canvas_arm_3 = MyMplCanvas(self, width=4, height=4, dpi=100)
        
      

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
        self.canvas_x = MyMplCanvas(self, width=5, height=4, dpi=100)
        self.canvas_y = MyMplCanvas(self, width=5, height=4, dpi=100)
        self.canvas_z = MyMplCanvas(self, width=5, height=4, dpi=100)
        self.canvas_roll = MyMplCanvas(self, width=5, height=4, dpi=100)
        self.canvas_pitch = MyMplCanvas(self, width=5, height=4, dpi=100)
        self.canvas_yaw = MyMplCanvas(self, width=5, height=4, dpi=100)

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

        # Initialize three lines for each plot (x, y, z, roll, pitch, yaw)
        self.x_lines = [self.canvas_x.axes.plot([], [])[0] for _ in range(1)]
        self.y_lines = [self.canvas_y.axes.plot([], [])[0] for _ in range(1)]
        self.z_lines = [self.canvas_z.axes.plot([], [])[0] for _ in range(1)]
        self.roll_lines = [self.canvas_roll.axes.plot([], [])[0] for _ in range(1)]
        self.pitch_lines = [self.canvas_pitch.axes.plot([], [])[0] for _ in range(1)]
        self.yaw_lines = [self.canvas_yaw.axes.plot([], [])[0] for _ in range(1)]

        self.ui.label_2.setText("Static Gait")
        self.ui.label_2.setStyleSheet("font-size: 40px; font-weight: bold; color: blue;")  # Set font size, weight, and color

      
        # QTimer to periodically update the plot
        self.plot_timer = QTimer(self)
        self.plot_timer.timeout.connect(self.plotFoots)
        self.plot_timer.start(500)  # Refresh plot every 100ms

        self.update_joint_positions()
        # Update the indicator based on conditions
        self.update_indicator()
    
    def update_indicator(self):
        """Update the indicator color or visibility based on some condition."""
        if self.robotMoving:
            self.indicator_quad.setStyleSheet("background-color: green")  # Change to green when moving
            self.indicator_arm.setStyleSheet("background-color: green")  # Change to green when moving
            self.indicator_leg.setStyleSheet("background-color: green")  # Change to green when moving
        else:
            self.indicator_quad.setStyleSheet("background-color: red")  # Change to red when stopped
            self.indicator_arm.setStyleSheet("background-color: red")  # Change to red when stopped
            self.indicator_leg.setStyleSheet("background-color: red")  # Change to red when stopped

    def plotFoots(self):
        self.update_indicator()
        try:
            self.update_plots()
        finally:
            pass
        footPose = self.ros_node.footPose
        cogPose = self.ros_node.cogPose
        quadMoving = self.robotMoving
        legMoving = self.leg

        
        footPose_2 =  self.ros_node.footPose_k

        EndEffector = [self.ros_node.endEfector[0],self.ros_node.endEfector[1]]
       
        self.plot_canvas.plot_circles(footPose[:],cogPose,quadMoving,legMoving,'Foots pose URDF')
        self.plot_canvas_2.plot_circles(footPose_2[:],cogPose,quadMoving,legMoving, 'Foots pose Kinematics')

        self.plot_canvas_arm.plot_circle(EndEffector , 'End Effector URDF XY','X axis','Y axis')
        EndEffector = [self.ros_node.endEfector[1],self.ros_node.endEfector[2]]
        self.plot_canvas_arm_2.plot_circle(EndEffector , 'End Effector URDF YZ','Y axis','Z axis')
        EndEffector = [self.ros_node.endEfector[0],self.ros_node.endEfector[2]]
        self.plot_canvas_arm_3.plot_circle(EndEffector, 'End Effector URDF XZ','X axis','Z axis')
       
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
    
    def moveRobot(self,angle,Force):
        currentAngle = 0
        self.robotMoving = not self.robotMoving
        
        if self.Robot == ARM:
            self.ros_node.msg_move.robot = "ARM"
            self.ros_node.msg_move.leg = "nn"
            
            current_angle = self.ros_node.arm.getJoint(self.armJoint)
        
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

            # Set updated joint angle
            self.ros_node.arm.setJoint(self.armJoint, new_angle)

            # Log updated joint angle for debugging
            updated_angle = self.ros_node.arm.getJoint(self.armJoint)
            self.ros_node.get_logger().info(f"Updated Arm Joint {self.armJoint} Angle: {updated_angle}")

            self.ros_node.msg_move.joint = str(self.armJoint)
            self.ros_node.msg_move.angle =  float(self.ros_node.arm.getJoint(self.armJoint) )
        elif self.Robot == QUAD:
            self.ros_node.msg_move.robot = 'QUAD'
            
            if self.leg == FL:
                
                # self.ros_node.get_logger().info(f"FL currentAngle  {currentAngle}")
                current_angle = self.ros_node.L_Front.getJoint(self.legJoint)     
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
                self.ros_node.L_Front.setJoint(self.legJoint,new_angle) 
                self.ros_node.msg_move.leg = str("FL")
                self.ros_node.msg_move.joint = str(self.legJoint)
                self.ros_node.msg_move.angle =  float(self.ros_node.L_Front.getJoint(self.legJoint))
            elif self.leg == FR:
                 # Calculate new joint angle
                current_angle = self.ros_node.R_Front.getJoint(self.legJoint) 
                if not Force:
                    if angle + current_angle <= 0:
                        new_angle = 0
                    elif angle + current_angle >= 3.142:
                        new_angle = 3.142
                    else:
                        new_angle = angle + current_angle
                else:
                    new_angle = angle 
                
                self.ros_node.R_Front.setJoint(self.legJoint,new_angle)   
                self.ros_node.msg_move.leg =str("FR")
                self.ros_node.msg_move.joint = str(self.legJoint)
                self.ros_node.msg_move.angle =  float(self.ros_node.R_Front.getJoint(self.legJoint))
            elif self.leg == BR:
                current_angle = self.ros_node.R_Back.getJoint(self.legJoint) 
                if not Force:
                    if angle + current_angle <= 0:
                        new_angle = 0
                    elif angle + current_angle >= 3.142:
                        new_angle = 3.142
                    else:
                        new_angle = angle + current_angle
                else:
                    new_angle = angle 
                
                self.ros_node.R_Back.setJoint(self.legJoint,new_angle) 
                self.ros_node.msg_move.leg =str("FR")
                self.ros_node.msg_move.joint = str(self.legJoint)
                self.ros_node.msg_move.angle =  float(self.ros_node.R_Back.getJoint(self.legJoint))
            elif self.leg == BL:
               
                current_angle = self.ros_node.L_Back.getJoint(self.legJoint) 
                if not Force:
                    if angle + current_angle <= 0:
                        new_angle = 0
                    elif angle + current_angle >= 3.142:
                        new_angle = 3.142
                    else:
                        new_angle = angle + current_angle
                else:
                    new_angle = angle 
                
                self.ros_node.L_Back.setJoint(self.legJoint,new_angle)
                self.ros_node.msg_move.leg =str("FR")
                self.ros_node.msg_move.joint = str(self.legJoint)
                self.ros_node.msg_move.angle =  float(self.ros_node.L_Back.getJoint(self.legJoint))

        self.update_joint_positions()
        self.publishMessage()
    def update_joint_positions(self):
        # Update the arm joint positions
        position = [
            
            float(0),
            float(0),
            float(0) ,                                  
            float(0),
            float(0),
            float(0),


            # Leg FL (Front Left)
            
            float(self.ros_node.L_Front.getJoint(Joint_0)),  # Update 'joint_0' of the FL leg
            float(self.ros_node.L_Front.getJoint(Joint_1)),  # Update 'joint_0' of the FL leg
            float(self.ros_node.L_Front.getJoint(Joint_2)),  # Update 'joint_0' of the FL leg
            
            # Leg FR (Front Right)
            
            float(self.ros_node.R_Front.getJoint(Joint_0)),  # Update 'joint_0' of the FL leg
            float(self.ros_node.R_Front.getJoint(Joint_1)),  # Update 'joint_0' of the FL leg
            float(self.ros_node.R_Front.getJoint(Joint_2)),  # Update 'joint_0' of the FL leg
            
            # Leg BL (Back Right)
            
            float(self.ros_node.R_Back.getJoint(Joint_0)),  # Update 'joint_0' of the FL leg
            float(self.ros_node.R_Back.getJoint(Joint_1)),  # Update 'joint_0' of the FL leg
            float(self.ros_node.R_Back.getJoint(Joint_2)),  # Update 'joint_0' of the FL leg
            
            # Leg BR (Back Right)
           
            float(self.ros_node.L_Back.getJoint(Joint_0)),  # Update 'joint_0' of the FL leg
            float(self.ros_node.L_Back.getJoint(Joint_1)),  # Update 'joint_0' of the FL leg
            float(self.ros_node.L_Back.getJoint(Joint_2)),  # Update 'joint_0' of the FL leg

            float(self.ros_node.arm.getJoint(Joint_0)),  # Update ARM
            float(self.ros_node.arm.getJoint(Joint_1)),  # Update 'joint_0' of the FL leg
            float(self.ros_node.arm.getJoint(Joint_2)),  # Update 'joint_0' of the FL leg
            float(self.ros_node.arm.getJoint(Joint_3)) ,  # Update 'joint_0' of the FL leg
            float(self.ros_node.arm.getJoint(Joint_4))  # Update 'joint_0' of the FL leg
            
        ]

        # Ensure the JointState message header timestamp is updated

        self.ros_node.joint_state_msg.position = [float(value) for value in  position ]

        self.ros_node.joint_state_msg.header.stamp = self.ros_node.get_clock().now().to_msg()
        
        # Publish the updated joint states
        self.ros_node.joint_states_pub.publish(self.ros_node.joint_state_msg)

    def publishMessage(self):
        self.ros_node.msg_move.angle =  float(self.ros_node.msg_move.angle )*180/np.pi
        self.ros_node.publisher.publish(self.ros_node.msg_move)

    
    def update_plots(self):
        """Update the plots with new data from ROS."""
        
        # Create an x-axis for each plot, based on the number of data points
        x_axis_x = range(1000)
        x_axis_y = range(1000)
        x_axis_z = range(1000)
        x_axis_roll = range(1000)
        x_axis_pitch = range(1000)
        x_axis_yaw = range(1000)
        """
         # Debugging: Print the lengths and types of the data before plotting
        self.ros_node.get_logger().info(f"Data X Length: {len(self.ros_node.data_x)}, Data Y Length: {len(self.ros_node.data_y)}")
        self.ros_node.get_logger().info(f"Data Z Length: {len(self.ros_node.data_z)}, Data Roll Length: {len(self.ros_node.data_roll)}")
        self.ros_node.get_logger().info(f"Data Pitch Length: {len(self.ros_node.data_pitch)}, Data Yaw Length: {len(self.ros_node.data_yaw)}")
        self.ros_node.get_logger().info(f"First 5 X Values: {self.ros_node.data_x[:5]}")
        self.ros_node.get_logger().info(f"First 5 Y Values: {self.ros_node.data_y[:5]}")
            # Ensure the data is non-empty and numeric before plotting
            # Ensure that x and y data arrays have the same length
        """
        if len(x_axis_x) == len(self.ros_node.data_x):
            self.x_lines[0].set_data(x_axis_x, self.ros_node.data_x)
        else:
            return

        if len(x_axis_y) == len(self.ros_node.data_y):
            self.y_lines[0].set_data(x_axis_y, self.ros_node.data_y)
        else:
            return

        if len(x_axis_z) == len(self.ros_node.data_z):
            self.z_lines[0].set_data(x_axis_z, self.ros_node.data_z)
        else:
            return

        if len(x_axis_roll) == len(self.ros_node.data_roll):
            self.roll_lines[0].set_data(x_axis_roll, self.ros_node.data_roll)
        else:
            return

        if len(x_axis_pitch) == len(self.ros_node.data_pitch):
            self.pitch_lines[0].set_data(x_axis_pitch, self.ros_node.data_pitch)
        else:
            return

        if len(x_axis_yaw) == len(self.ros_node.data_yaw):
            self.yaw_lines[0].set_data(x_axis_yaw, self.ros_node.data_yaw)
        else:
            return
        if not self.ros_node.data_x or not self.ros_node.data_y or not self.ros_node.data_z:
            return
        if not self.ros_node.data_roll or not self.ros_node.data_pitch or not self.ros_node.data_yaw:
            return
             # Adjust plot limits and redraw each plot
        self.canvas_x.axes.set_xlabel('Time (s)')
        self.canvas_x.axes.set_ylabel('Position X (m)')
        self.canvas_x.axes.set_title('Position X vs Time')
        self.canvas_x.axes.relim()
        self.canvas_x.axes.autoscale_view()
        self.canvas_x.fig.tight_layout()
        self.canvas_x.fig.subplots_adjust(left=0.2, right=0.8, top=0.8, bottom=0.2)
        self.canvas_x.axes.grid(True)
        self.canvas_x.draw()

        self.canvas_y.axes.set_xlabel('Time (s)')
        self.canvas_y.axes.set_ylabel('Position Y (m)')
        self.canvas_y.axes.set_title('Position Y vs Time')
        self.canvas_y.fig.tight_layout()
        self.canvas_y.fig.subplots_adjust(left=0.2, right=0.8, top=0.8, bottom=0.2)
        self.canvas_y.axes.relim()
        self.canvas_y.axes.autoscale_view()
        self.canvas_y.axes.grid(True)
        self.canvas_y.draw()

        self.canvas_z.axes.set_xlabel('Time (s)')
        self.canvas_z.axes.set_ylabel('Position Z (m)')
        self.canvas_z.axes.set_title('Position Z vs Time')
        self.canvas_z.fig.tight_layout()
        self.canvas_z.fig.subplots_adjust(left=0.2, right=0.8, top=0.8, bottom=0.2)
        self.canvas_z.axes.relim()
        self.canvas_z.axes.autoscale_view()
        self.canvas_z.axes.grid(True)
        self.canvas_z.draw()

        self.canvas_roll.axes.set_xlabel('Time (s)')
        self.canvas_roll.axes.set_ylabel('Orientation roll (rad)')
        self.canvas_roll.axes.set_title('Roll vs Time')
        self.canvas_roll.fig.tight_layout()
        self.canvas_roll.fig.subplots_adjust(left=0.2, right=0.8, top=0.8, bottom=0.2)
        self.canvas_roll.axes.relim()
        self.canvas_roll.axes.autoscale_view()
        self.canvas_roll.axes.grid(True)
        self.canvas_roll.draw()

        self.canvas_pitch.axes.set_xlabel('Time (s)')
        self.canvas_pitch.axes.set_ylabel('Orientation pitch (rad)')
        self.canvas_pitch.axes.set_title('Pitchvs Time')
        self.canvas_pitch.fig.tight_layout()
        self.canvas_pitch.fig.subplots_adjust(left=0.2, right=0.8, top=0.8, bottom=0.2)
        self.canvas_pitch.axes.relim()
        self.canvas_pitch.axes.autoscale_view()
        self.canvas_pitch.axes.grid(True)
        self.canvas_pitch.draw()

        self.canvas_yaw.axes.set_xlabel('Time (s)')
        self.canvas_yaw.axes.set_ylabel('Orientation yaw (rad)')
        self.canvas_yaw.axes.set_title('Yaw vs Time')
        self.canvas_yaw.fig.tight_layout()
        self.canvas_yaw.fig.subplots_adjust(left=0.2, right=0.8, top=0.8, bottom=0.2)
        self.canvas_yaw.axes.relim()
        self.canvas_yaw.axes.autoscale_view()
        self.canvas_yaw.axes.grid(True)
        self.canvas_yaw.draw()
        # Plot the circles on the canvas
    

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