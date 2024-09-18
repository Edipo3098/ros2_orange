import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from threading import Thread
from robot_interfaces.msg import Mpu
import sys
from Quad_control_gui.control import Ui_Dialog


class MyNode(Node):
    def __init__(self, label):
        super().__init__('control_node')
        self.subscription = self.create_subscription(
            Mpu,
            'mpu_data',
            self.listener_callback,
            10
        )
        self.data1 = 0
        self.data2 = 0
        self.publisher = self.create_publisher(String, 'output_topic', 10)
        self.label = label  # Label object from the UI

    def listener_callback(self, msg):
        """
        Callback function triggered when a message is received on the 'mpu_data' topic.
        The label in the UI will be updated with the message data.
        """
        self.label.setText(f"Received: {msg.gz}")  # Update the UI label with received data
        self.data1 = msg.gz
        self.data2 = msg.gy
    def publish_message(self, message):
        """
        Publishes a string message to the 'output_topic'.
        """
        msg = String()
        msg.data = message
        self.publisher.publish(msg)

class MyWindow(QMainWindow):
    def __init__(self, ros_node):
        super(MyWindow, self).__init__()
        self.ui = Ui_Dialog()  # Instantiate the UI class generated from the .ui file
        self.ui.setupUi(self)      # Set up the UI and bind objects from the .ui file

        # Connect button 1 to its corresponding function
        self.ui.pushButton_1.clicked.connect(self.on_button_1_click)
        
        # Connect button 2 to its corresponding function
        self.ui.pushButton_2.clicked.connect(self.on_button_2_click)

        # Reference to the ROS2 node for interacting with ROS topics
        self.ros_node = ros_node

    def on_button_1_click(self):
        """
        This function is called when button 1 is clicked. It publishes a ROS2 message
        and updates the label in the UI.
        """
        self.ros_node.publish_message("Button 1 Clicked")  # Publish message to ROS2 topic
        self.ui.label.setText(f"gz: {self.ros_node.data1}")

    def on_button_2_click(self):
        """
        This function is called when button 2 is clicked. It publishes a ROS2 message
        and updates the label in the UI.
        """
        self.ros_node.publish_message("Button 2 Clicked")  # Publish message to ROS2 topic
        self.ui.label.setText(f"gz: {self.ros_node.data2}")

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
    label = QLabel()  # Create a QLabel object that will be updated in the ROS node
    ros_node = MyNode(label)  # Create the ROS2 node with the QLabel passed in

    # Create the PyQt5 window and pass the ROS2 node to it
    window = MyWindow(ros_node)
    window.show()

    # Run the ROS2 node in a separate thread so that it doesn't block the UI thread
    ros_thread = Thread(target=ros_spin_thread, args=(ros_node,), daemon=True)
    ros_thread.start()

    # Ensure ROS2 shutdown when PyQt application exits
    app.aboutToQuit.connect(lambda: ros_node.destroy_node())
    app.aboutToQuit.connect(lambda: rclpy.shutdown())

    sys.exit(app.exec_())  # Start the PyQt5 application event loop

if __name__ == '__main__':
    main()
