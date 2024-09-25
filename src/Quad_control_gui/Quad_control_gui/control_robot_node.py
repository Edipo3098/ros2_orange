import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QWidget, QStackedWidget
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from robot_interfaces.msg import Anglemotor
from robot_interfaces.msg import Command
from threading import Thread
from robot_interfaces.msg import Mpu
from sensor_msgs.msg import JointState
import sys
from Quad_control_gui.control_robot import Ui_RobotControl
from enum import Enum
ARM = 0
QUAD = 1
FL = 0
FR = 1
BL = 2
BR = 3
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
    backLeft_hip_joint = 'backLeft_hip_joint'
    backLeft_hip_motor_joint = 'backLeft_hip_motor_joint'
    backLeft_knee_joint = 'backLeft_knee_joint'
    backLeft_ankle_joint = 'backLeft_ankle_joint'
    backRight_hip_joint = 'backRight_hip_joint'
    backRight_hip_motor_joint = 'backRight_hip_motor_joint'
    backRight_knee_joint = 'backRight_knee_joint'
    backRight_ankle_joint = 'backRight_ankle_joint'
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
        self.joint_state_msg = JointState()
         # List of joint names from your URDF
        self.joint_state_msg.name = self.joint_state_msg.name = [joint.value for joint in Joints]
        
        self.joint_state_msg.position = [float(value) for value in [0, 0, 0, 0, 0, 0, 0, 0, 0.5, -1, 0, 0, 0.5, -1, 0, 0, 0.5, -1, 0, 0, 0.5, -1, 0, 0, 0, 0, 0]]

        # Set the current time for the header
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the initial joint states
        
        self.get_logger().info('Initial joint states published.')
        self.joint_states_pub.publish(self.joint_state_msg)
        self.RobotReady = False
        
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
        self.LegFR = { 'joint_0':0 , 'joint_1':0.5 , 'joint_2':-1}
        self.LegBL = { 'joint_0':0 , 'joint_1':0.5 , 'joint_2':-1}
        self.LegFL = { 'joint_0':0 , 'joint_1':0.5 , 'joint_2':-1}
        self.LegBR = { 'joint_0':0 , 'joint_1':0.5 , 'joint_2':-1}
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
                    if angle - currentAngle <= 0:
                        self.LegFL[self.legJoint] = 0
                    elif angle+currentAngle >= 3.142:
                        self.LegFL[self.legJoint] = 3.142
                    else:
                        self.LegFL[self.legJoint] = angle 
                else: 
                    self.LegFL[self.legJoint] = angle                
                self.ros_node.msg_move.leg = str("FL")
                self.ros_node.msg_move.joint = self.legJoint
                self.ros_node.msg_move.leg =  float(self.LegFL[self.legJoint])
            elif self.leg == BL:
                if not Force:
                    currentAngle = self.LegBL[self.legJoint] 
                    if angle - currentAngle <= 0:
                        self.LegBL[self.legJoint] = 0
                    elif angle+currentAngle >= 3.142:
                        self.LegBL[self.legJoint] = 3.142
                    else:
                        self.LegBL[self.legJoint] = angle + currentAngle
                else:
                    self.LegBL[self.legJoint] = angle 
                self.ros_node.msg_move.leg = 'BL'
                self.ros_node.msg_move.joint = self.legJoint
                self.ros_node.msg_move.leg =  float(self.LegBL[self.legJoint])
            elif self.leg == FR:
                if not Force:
                    currentAngle = self.LegFR[self.legJoint] 
                    if angle - currentAngle <= 0:
                        self.LegFR[self.legJoint] = 0
                    elif angle+currentAngle >= 3.142:
                        self.LegFR[self.legJoint] = 3.142
                    else:
                        self.LegFR[self.legJoint] = angle + currentAngle
                else:
                    self.LegFR[self.legJoint] = angle
                self.ros_node.msg_move.leg = 'FR'
                self.ros_node.msg_move.joint = self.legJoint
                self.ros_node.msg_move.leg =  float(self.LegFR[self.legJoint])
            elif self.leg == BR:
                if not Force:
                    currentAngle = self.LegBR[self.legJoint] 
                    if angle - currentAngle <= 0:
                        self.LegBR[self.legJoint] = 0
                    elif angle+currentAngle >= 3.142:
                        self.LegBR[self.legJoint] = 3.142
                    else:
                        self.LegBR[self.legJoint] = angle + currentAngle
                else:
                    self.LegBR[self.legJoint] = angle 

                self.ros_node.msg_move.leg = 'BR'
                self.ros_node.msg_move.joint = self.legJoint
                self.ros_node.msg_move.leg =  float(self.LegBR[self.legJoint])

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
            self.LegFL['joint_0'],  # Update 'joint_0' of the FL leg
            self.LegFL['joint_1'],  # Update 'joint_1' of the FL leg
            self.LegFL['joint_2'],  # Update 'joint_2' of the FL leg
            
            # Leg FR (Front Right)
            self.LegFR['joint_0'],  # Update 'joint_0' of the FR leg
            self.LegFR['joint_1'],  # Update 'joint_1' of the FR leg
            self.LegFR['joint_2'],  # Update 'joint_2' of the FR leg
            
            # Leg BL (Back Left)
            self.LegBL['joint_0'],  # Update 'joint_0' of the BL leg
            self.LegBL['joint_1'],  # Update 'joint_1' of the BL leg
            self.LegBL['joint_2'],  # Update 'joint_2' of the BL leg
            
            # Leg BR (Back Right)
            self.LegBR['joint_0'],  # Update 'joint_0' of the BR leg
            self.LegBR['joint_1'],  # Update 'joint_1' of the BR leg
            self.LegBR['joint_2'],   # Update 'joint_2' of the BR leg

            self.jointArm['joint_0'],  # Update 'joint_0' of the arm
            self.jointArm['joint_1'],  # Update 'joint_1' of the arm
            self.jointArm['joint_2'],  # Update 'joint_2' of the arm
            self.jointArm['joint_3'],  # Update 'joint_3' of the arm
            self.jointArm['joint_4']  # Update 'joint_4' of the arm
        ]

        # Ensure the JointState message header timestamp is updated

        self.ros_node.joint_state_msg.position = [float(value) for value in  position ]

        self.ros_node.joint_state_msg.stamp = self.get_clock().now().to_msg()
        
        # Publish the updated joint states
        self.ros_node.joint_states_pub.publish(self.joint_state_msg)

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