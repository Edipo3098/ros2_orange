import sys
from threading import Thread
from enum import Enum

from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QWidget, QStackedWidget,QVBoxLayout
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.patches import Circle
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor
from PyQt5.QtCore import QTimer


import rclpy
from rclpy.node import Node


from Quad_control_gui.control_robot import Ui_RobotControl
from tf2_ros import TransformListener, Buffer

from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from robot_interfaces.msg import Anglemotor
from robot_interfaces.msg import Command
from robot_interfaces.msg import Mpu
from sensor_msgs.msg import JointState

ARM = 0
QUAD = 1

FL = 0
FR = 1
BR = 2
BL = 3
class Joints(Enum):
    X_joint = 'X_joint'
    Y_joint = 'Y_joint'
    Z_joint = 'Z_joint'
    Roll_joint = 'Roll_joint'
    Pitch_joint = 'Pitch_joint'
    Yaw_joint = 'Yaw_joint'
    frontLeft_hip_joint = 'frontLeft_hip_joint'
    frontLeft_hip_motor_joint = 'frontLeft_hip_motor_joint'
    frontLeft_knee_joint = 'frontLeft_knee_joint'
    frontLeft_ankle_joint = 'frontLeft_ankle_joint'
    frontRight_hip_joint = 'frontRight_hip_joint'
    frontRight_hip_motor_joint = 'frontRight_hip_motor_joint'
    frontRight_knee_joint = 'frontRight_knee_joint'
    frontRight_ankle_joint = 'frontRight_ankle_joint'
    backRight_hip_joint = 'backRight_hip_joint'
    backRight_hip_motor_joint = 'backRight_hip_motor_joint'
    backRight_knee_joint = 'backRight_knee_joint'
    backRight_ankle_joint = 'backRight_ankle_joint'
    backLeft_hip_joint = 'backLeft_hip_joint'
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
        self.publisher = self.create_publisher(Anglemotor, 'motor_angles', 10)
        self.subscription = self.create_subscription(Command, 'command_robot', self.listener_callback, 10)
        self.joint_states_pub = self.create_publisher(JointState,'joint_states',10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to periodically check the transform
        self.timer_footPose = self.create_timer(0.1, self.timer_foot_callback)

        self.joint_state_msg = JointState()
         # List of joint names from your URDF
        self.joint_state_msg.name = self.joint_state_msg.name = [joint.value for joint in Joints]
        
        self.joint_state_msg.position = [float(value) for value in [0, 0, 0, 0, 0, 0, 0, 0, 0.6, -1.7, 0, 0, 0.6, -1.7, 0, 0, 0.6, -1.7, 0, 0, 0.6, -1.7, 0, 0, 0, 0, 0]]

        # Set the current time for the header
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the initial joint states
        
        self.get_logger().info('Initial joint states published.')
        self.joint_states_pub.publish(self.joint_state_msg)
        self.RobotReady = False
        self.footPose = [[0,0],[0,0],[0,0],[0,0]]
        self.cogPose = [0,0]
    def timer_foot_callback(self):
        foot= ['frontLeft_foot','frontRight_foot','backRight_foot','backLeft_foot'] 
        x = 0
        for i in foot:
            
            try:
                
                # Look for the transform from the world frame to the end effector
                now = rclpy.time.Time()
                transform = self.tf_buffer.lookup_transform('odom', i, now)
                
                # Extract the translation (position) and rotation (orientation)
                trans = transform.transform.translation
                rot = transform.transform.rotation  
                
                # Print the position and orientation of the end effector
                self.get_logger().info(f"{i}: x={trans.x}, y={trans.y}, z={trans.z}")
                
                self.footPose[x] = [trans.x ,trans.y]
               

                x +=1
            
            except Exception as e:
                self.get_logger().warn(f"Could not get transform: {str(e)}")
         
            # Look for the transform from the world frame to the end effector
            try:
                now = rclpy.time.Time()
                transform = self.tf_buffer.lookup_transform('odom', 'Yaw_body', now)
                
                # Extract the translation (position) and rotation (orientation)
                trans = transform.transform.translation
                rot = transform.transform.rotation  
                
                # Print the position and orientation of the end effector
                self.get_logger().info(f"{i}: x={trans.x}, y={trans.y}, z={trans.z}")
                
                self.cogPose = [trans.x ,trans.y]
            except Exception as e:
                self.get_logger().warn(f"Could not get transform: {str(e)}")

        
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
        msg = String()
        msg.data = message
        self.publisher.publish(msg)
    def publish_joint_states(self):
        """
        Publishes joint states to the /joint_states topic
        """
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        # Here you update the joint states with the actual positions
        # self.joint_state_msg.position[0] = <your_value>

        # Publish the joint states
        self.joint_states_pub.publish(self.joint_state_msg)

class MyMplCanvas(FigureCanvas):
    """A Matplotlib canvas that can be embedded into a PyQt5 application."""
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)  # Create a single subplot
        super(MyMplCanvas, self).__init__(fig)

    def plot_circles(self,footPose,cogPose,quadMoving,legMoving):
        """Plot five circles at specified positions."""
        # Define circle centers
        centers = footPose
        cogCenter = cogPose
        radius = 0.05  # Circle radius

        # Clear the axes
        self.axes.clear()
        self.axes.cla()
        
       # Plot each circle using matplotlib's Circle patch
        for [x0, y0] in centers:
            circle = Circle((x0, y0), radius, color='blue', fill=False)  # Create a circle using Circle from patches
            self.axes.add_patch(circle)
        # If there are multiple centers, connect them with a line
        circle = Circle((cogCenter[0], cogCenter[1]), radius, color='red', fill=True)  # Create a circle using Circle from patches
        self.axes.add_patch(circle)
            
        if quadMoving:
            centers.pop(legMoving)
            
        x_coords = [center[0] for center in centers]
        y_coords = [center[1] for center in centers]

        x_coords.append(x_coords[0])
        y_coords.append(y_coords[0])
            # Plot the line connecting the centers
        self.axes.plot(x_coords, y_coords, color='red', linestyle='-', linewidth=2)


        # Set axis limits and aspect ratio
        self.axes.set_xlim(-1, 2)
        self.axes.set_ylim(-1, 2)
        self.axes.set_aspect('equal')

        # Re-draw the plot
        self.draw()

class MyWindow(QMainWindow):
    def __init__(self, ros_node):
        super(MyWindow, self).__init__()
        self.resize(800, 600)  # Set the size of the main window

        self.ui = Ui_RobotControl()  # Instantiate the UI class generated from the .ui file
        # Create a central widget to hold your stacked widget
        
        stacked_widget = QStackedWidget()  # This should be a QStackedWidget, not a generic QWidget
        self.ui.setupUi(stacked_widget)
        # Set the QStackedWidget as the central widget of the QMainWindow
        self.setCentralWidget(stacked_widget)  # THIS IS THE CRUCIAL LINE

        self.stacked_widget = stacked_widget  
        self.stacked_widget.setCurrentIndex(0)
        self.ui.controlBasic.setVisible(True)
        self.stacked_widget.update()
         # Embed the Matplotlib plot in the controlQuad page (index 1 of the QStackedWidget)
       
        
        # Connect comboBox to switch pages in the stacked widget
        self.ui.changeRobot.currentIndexChanged.connect(self.change_robot)
        self.ui.jointArm.currentIndexChanged.connect(self.change_jointArm)
        self.ui.jointLegBL.currentIndexChanged.connect(self.change_jointLeg_BL)
        self.ui.jointLegFR.currentIndexChanged.connect(self.change_jointLeg_FR)
        self.ui.jointLegBR.currentIndexChanged.connect(self.change_jointLeg_BR)
        self.ui.jointLegFL.currentIndexChanged.connect(self.change_jointLeg_FL)

        self.ui.nextPage.clicked.connect(self.nextPage)
        self.ui.nextPage2.clicked.connect(self.nextPage)
        self.ui.prevButtom2.clicked.connect(self.prevPage)
        self.ui.nextPage3.clicked.connect(self.nextPage)
        self.ui.prevButtom3.clicked.connect(self.prevPage)
        self.currentIdx = 0
        # Connect buttons to corresponding functions
        self.ui.setCero.clicked.connect(self.on_set_zero_click)
        self.ui.setMinus10.clicked.connect(self.on_minus_10_click)
        self.ui.setPLus10.clicked.connect(self.on_plus_10_click)
        self.ui.setMax.clicked.connect(self.on_set_max_click)

        self.jointArm = { 'joint_0':0 , 'joint_1':0 , 'joint_2':0 , 'joint_3':0 , 'joint_4':0  }
        self.LegFR = { 'joint_0':0 , 'joint_1':0.6 , 'joint_2':-1.7}
        self.LegBL = { 'joint_0':0 , 'joint_1':0.6 , 'joint_2':-1.7}
        self.LegFL = { 'joint_0':0 , 'joint_1':0.6 , 'joint_2':-1.7}
        self.LegBR = { 'joint_0':0 , 'joint_1':0.6 , 'joint_2':-1.7}
        # Reference to the ROS2 node for interacting with ROS topics
        self.robotMoving = False
        
        self.armJoint = 'joint_0'
        self.leg = FL
        self.legJoint = 'joint_0'
        self.Robot = ARM
        self.ui.jointLegBL.setVisible(False)
        self.ui.jointLegBR.setVisible(False)
        self.ui.jointLegFL.setVisible(False)
        self.ui.jointLegFR.setVisible(False)
        self.ui.jointArm.setVisible(True)
        self.ros_node = ros_node


        self.plot_canvas = MyMplCanvas(self, width=5, height=4, dpi=100)

        # Find the layout of the plotFoot widget (the placeholder in your UI on controlQuad)
        layout = QVBoxLayout(self.ui.plotFoot)  # plotFoot is in the controlQuad page
        layout.addWidget(self.plot_canvas)      # Add the matplotlib canvas to the layout
        # QTimer to periodically update the plot
        self.plot_timer = QTimer(self)
        self.plot_timer.timeout.connect(self.plotFoots)
        self.plot_timer.start(100)  # Refresh plot every 100ms
        
        # Plot the circles on the canvas
        
    def plotFoots(self):
        footPose = self.ros_node.footPose
        cogPose = self.ros_node.cogPose
        quadMoving = self.robotMoving
        legMoving = self.leg
        self.plot_canvas.plot_circles(footPose[:],cogPose,quadMoving,legMoving)

    def nextPage(self):
        if self.currentIdx < 2:
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
            self.ui.jointLegBL.setVisible(False)
            self.ui.jointLegBR.setVisible(False)
            self.ui.jointLegFL.setVisible(False)
            self.ui.jointLegFR.setVisible(False)
            self.ui.jointArm.setVisible(True)
        elif index == 1:
            self.Robot = QUAD
            self.ui.jointLegBL.setVisible(True)
            self.ui.jointLegBR.setVisible(True)
            self.ui.jointLegFL.setVisible(True)
            self.ui.jointLegFR.setVisible(True)
            self.ui.jointArm.setVisible(False)
    def change_jointArm(self, index):
        """
        Change joint to move
        """
        
        print(f"Changing joint {index}")
        match index:
            case 0:
                self.armJoint = 'joint_0'
            case 1:
                self.armJoint = 'joint_1'
            case 2:
                self.armJoint = 'joint_2'
            case 3:
                self.armJoint = 'joint_3'
            case 4:
                self.armJoint = 'joint_4'


    def change_jointLeg_BL(self, index):
        """
        Change joint to move
        """
        print(f"Changing joint BL {index}")
        self.leg = BL
        match index:
            case 0:
                self.legJoint = 'joint_0'
            case 1:
                self.legJoint = 'joint_1'
            case 2:
                self.legJoint = 'joint_2'
    def change_jointLeg_FR(self, index):
        """
        Change joint to move
        """
        print(f"Changing joint  FR {index}")
        self.leg = FR
        match index:
            case 0:
                self.legJoint = 'joint_0'
            case 1:
                self.legJoint = 'joint_1'
            case 2:
                self.legJoint = 'joint_2'
    def change_jointLeg_BR(self, index):
        """
        Change joint to move
        """
        print(f"Changing joint BR {index}")
        self.leg = BR
        match index:
            case 0:
                self.legJoint = 'joint_0'
            case 1:
                self.legJoint = 'joint_1'
            case 2:
                self.legJoint = 'joint_2'
    def change_jointLeg_FL(self, index):
        """
        Change joint to move
        """
        print(f"Changing joint  FL {index}")
        self.leg = FL
        match index:
            case 0:
                self.legJoint = 'joint_0'
            case 1:
                self.legJoint = 'joint_1'
            case 2:
                self.legJoint = 'joint_2'
        

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
            currentAngle = self.jointArm[self.armJoint]
            if not Force:
                if angle + currentAngle <= 0:
                    self.jointArm[self.armJoint] = 0
                elif angle+currentAngle >= 3.142:
                    self.jointArm[self.armJoint] = 3.142
                else:
                    self.jointArm[self.armJoint] = angle+currentAngle
            else:
                self.jointArm[self.armJoint] = angle
            
            print("Current angle %f",self.jointArm[self.armJoint])
            self.ros_node.msg_move.joint = self.armJoint
            self.ros_node.msg_move.angle =  float(self.jointArm[self.armJoint])
        elif self.Robot == QUAD:
            self.ros_node.msg_move.robot = 'QUAD'
            
            if self.leg == FL:
                if not Force:
                    currentAngle = self.LegFL[self.legJoint] 
                    if angle + currentAngle <= -3.142:
                        self.LegFL[self.legJoint] = 0
                    elif angle+currentAngle >= 3.142:
                        self.LegFL[self.legJoint] = 3.142
                    else:
                        self.LegFL[self.legJoint] = angle +currentAngle
                else: 
                    self.LegFL[self.legJoint] = angle                
                self.ros_node.msg_move.leg = str("FL")
                self.ros_node.msg_move.joint = self.legJoint
                self.ros_node.msg_move.angle =  float(self.LegFL[self.legJoint])
            elif self.leg == BL:
                if not Force:
                    currentAngle = self.LegBL[self.legJoint] 
                    if angle + currentAngle <= -3.142:
                        self.LegBL[self.legJoint] = 0
                    elif angle+currentAngle >= 3.142:
                        self.LegBL[self.legJoint] = 3.142
                    else:
                        self.LegBL[self.legJoint] = angle + currentAngle
                else:
                    self.LegBL[self.legJoint] = angle 
                self.ros_node.msg_move.leg = 'BL'
                self.ros_node.msg_move.joint = self.legJoint
                self.ros_node.msg_move.angle =  float(self.LegBL[self.legJoint])
            elif self.leg == FR:
                if not Force:
                    currentAngle = self.LegFR[self.legJoint] 
                    if angle + currentAngle <= -3.142:
                        self.LegFR[self.legJoint] = 0
                    elif angle+currentAngle >= 3.142:
                        self.LegFR[self.legJoint] = 3.142
                    else:
                        self.LegFR[self.legJoint] = angle + currentAngle
                else:
                    self.LegFR[self.legJoint] = angle
                self.ros_node.msg_move.leg = 'FR'
                self.ros_node.msg_move.joint = self.legJoint
                self.ros_node.msg_move.angle =  float(self.LegFR[self.legJoint])
            elif self.leg == BR:
                if not Force:
                    currentAngle = self.LegBR[self.legJoint] 
                    if angle + currentAngle <= -3.142:
                        self.LegBR[self.legJoint] = 0
                    elif angle+currentAngle >= 3.142:
                        self.LegBR[self.legJoint] = 3.142
                    else:
                        self.LegBR[self.legJoint] = angle + currentAngle
                else:
                    self.LegBR[self.legJoint] = angle 

                self.ros_node.msg_move.leg = 'BR'
                self.ros_node.msg_move.joint = self.legJoint
                self.ros_node.msg_move.angle =  float(self.LegBR[self.legJoint])

        self.update_joint_positions()
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
            float(0),
            self.LegFL['joint_0'],  # Update 'joint_0' of the FL leg
            self.LegFL['joint_1'],  # Update 'joint_1' of the FL leg
            self.LegFL['joint_2'],  # Update 'joint_2' of the FL leg
            
            # Leg FR (Front Right)
            float(0),
            self.LegFR['joint_0'],  # Update 'joint_0' of the FR leg
            self.LegFR['joint_1'],  # Update 'joint_1' of the FR leg
            self.LegFR['joint_2'],  # Update 'joint_2' of the FR leg
            
            # Leg BL (Back Right)
            float(0),
            self.LegBR['joint_0'],  # Update 'joint_0' of the BL leg
            self.LegBR['joint_1'],  # Update 'joint_1' of the BL leg
            self.LegBR['joint_2'],  # Update 'joint_2' of the BL leg
            
            # Leg BR (Back Right)
            float(0),
            self.LegBL['joint_0'],  # Update 'joint_0' of the BR leg
            self.LegBL['joint_1'],  # Update 'joint_1' of the BR leg
            self.LegBL['joint_2'],   # Update 'joint_2' of the BR leg

            self.jointArm['joint_0'],  # Update 'joint_0' of the arm
            self.jointArm['joint_1'],  # Update 'joint_1' of the arm
            self.jointArm['joint_2'],  # Update 'joint_2' of the arm
            self.jointArm['joint_3'],  # Update 'joint_3' of the arm
            self.jointArm['joint_4']  # Update 'joint_4' of the arm
        ]

        # Ensure the JointState message header timestamp is updated

        self.ros_node.joint_state_msg.position = [float(value) for value in  position ]

        self.ros_node.joint_state_msg.header.stamp = self.ros_node.get_clock().now().to_msg()
        
        # Publish the updated joint states
        self.ros_node.joint_states_pub.publish(self.ros_node.joint_state_msg)

    def publishMessage(self):
        self.ros_node.publisher.publish(self.ros_node.msg_move)

    

    

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
    timer = ros_node.create_timer(0.1, ros_node.publish_joint_states)
    # Ensure ROS2 shutdown when PyQt application exits
    app.aboutToQuit.connect(lambda: ros_node.destroy_node())
    app.aboutToQuit.connect(lambda: rclpy.shutdown())
    sys.exit(app.exec_())  # Start the PyQt5 application event loop
if __name__ == '__main__':
    main()