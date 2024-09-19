import sys
from PyQt5.QtWidgets import QMainWindow, QApplication, QVBoxLayout, QWidget, QLabel
from PyQt5.QtCore import QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from threading import Thread
from robot_interfaces.msg import COGframe
from Quad_control_gui.control import Ui_Dialog


class MyMplCanvas(FigureCanvas):
    """A Matplotlib canvas embedded in PyQt."""
    def __init__(self, parent=None, width=5, height=4, dpi=1000):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super(MyMplCanvas, self).__init__(fig)


class MyNode(Node):
    def __init__(self, label):
        super().__init__('control_node')
        self.subscription = self.create_subscription(COGframe, 'kalman_cog_frame_3', self.listener_callback, 10)
        

        self.publisher = self.create_publisher(String, 'output_topic', 10)
        self.label = label  # QLabel object to display data in the UI

        # Sample data storage for x, y, z, roll, pitch, yaw with 3 sources for each
        self.data_x = []
        self.data_y = []
        self.data_z = []
        self.data_roll = []
        self.data_pitch = []
        self.data_yaw = []

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
            self.data_y.pop(0)
            self.data_z.pop(0)
            self.data_roll.pop(0)
            self.data_pitch.pop(0)
            self.data_yaw.pop(0) 
        self.get_logger().info('DATA RECEIVED')

        

    def listener_callback(self, msg):
        """Callback for cog_data topic."""
        self.add_data(msg, 0)

    

    def publish_message(self, message):
        """Publish a string message to the 'output_topic'."""
        msg = String()
        msg.data = message
        self.publisher.publish(msg)


class MyWindow(QMainWindow):
    def __init__(self, ros_node):
        super(MyWindow, self).__init__()
        self.ui = Ui_Dialog()  # Load the UI from the generated file
        self.ui.setupUi(self)  # Set up the UI

        # Reference to the ROS2 node for interacting with ROS topics
        self.ros_node = ros_node

        # Set up Matplotlib plots and curves for each axis (x, y, z, roll, pitch, yaw)
        self.setup_plots()

        # Timer to refresh the plots (e.g., every 100ms)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(100)

    def setup_plots(self):
        """Initialize Matplotlib figures for each degree of freedom."""
        # Create a layout and Matplotlib canvases for each axis (x, y, z, roll, pitch, yaw)
        layout = QVBoxLayout()

        self.canvas_x = MyMplCanvas(self, width=5, height=4, dpi=100)
        self.canvas_y = MyMplCanvas(self, width=5, height=4, dpi=100)
        self.canvas_z = MyMplCanvas(self, width=5, height=4, dpi=100)
        self.canvas_roll = MyMplCanvas(self, width=5, height=4, dpi=100)
        self.canvas_pitch = MyMplCanvas(self, width=5, height=4, dpi=100)
        self.canvas_yaw = MyMplCanvas(self, width=5, height=4, dpi=100)

        # Add each canvas to the layout
        layout.addWidget(self.canvas_x)
        layout.addWidget(self.canvas_y)
        layout.addWidget(self.canvas_z)
        layout.addWidget(self.canvas_roll)
        layout.addWidget(self.canvas_pitch)
        layout.addWidget(self.canvas_yaw)

        # Add the layout to the UI window (replace central widget with your custom layout)
        central_widget = QWidget()
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

        # Initialize three lines for each plot (x, y, z, roll, pitch, yaw)
        self.x_lines = [self.canvas_x.axes.plot([], [])[0] for _ in range(1)]
        self.y_lines = [self.canvas_y.axes.plot([], [])[0] for _ in range(1)]
        self.z_lines = [self.canvas_z.axes.plot([], [])[0] for _ in range(1)]
        self.roll_lines = [self.canvas_roll.axes.plot([], [])[0] for _ in range(1)]
        self.pitch_lines = [self.canvas_pitch.axes.plot([], [])[0] for _ in range(1)]
        self.yaw_lines = [self.canvas_yaw.axes.plot([], [])[0] for _ in range(1)]

    def update_plots(self):
        """Update the plots with new data from ROS."""
        
        # Create an x-axis for each plot, based on the number of data points
        x_axis_x = range(len(self.ros_node.data_x))
        x_axis_y = range(len(self.ros_node.data_y))
        x_axis_z = range(len(self.ros_node.data_z))
        x_axis_roll = range(len(self.ros_node.data_roll))
        x_axis_pitch = range(len(self.ros_node.data_pitch))
        x_axis_yaw = range(len(self.ros_node.data_yaw))
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

        if len(x_axis_y) == len(self.ros_node.data_y):
            self.y_lines[0].set_data(x_axis_y, self.ros_node.data_y)

        if len(x_axis_z) == len(self.ros_node.data_z):
            self.z_lines[0].set_data(x_axis_z, self.ros_node.data_z)

        if len(x_axis_roll) == len(self.ros_node.data_roll):
            self.roll_lines[0].set_data(x_axis_roll, self.ros_node.data_roll)

        if len(x_axis_pitch) == len(self.ros_node.data_pitch):
            self.pitch_lines[0].set_data(x_axis_pitch, self.ros_node.data_pitch)

        if len(x_axis_yaw) == len(self.ros_node.data_yaw):
            self.yaw_lines[0].set_data(x_axis_yaw, self.ros_node.data_yaw)

        """
        self.ros_node.get_logger().info(f"Data X Length: {len(self.ros_node.data_x)}, Data Y Length: {len(self.ros_node.data_y)}")

        self.ros_node.get_logger().info(f"Data z Length: {len(self.ros_node.data_z)}, Data R Length: {len(self.ros_node.data_roll)}")

        self.ros_node.get_logger().info(f"Data P Length: {len(self.ros_node.data_pitch)}, Data W Length: {len(self.ros_node.data_yaw)}")
        """
        try:
            # Adjust plot limits and redraw each plot
            self.canvas_x.axes.relim()
            self.canvas_x.axes.autoscale_view()
            self.canvas_x.draw()

            self.canvas_y.axes.relim()
            self.canvas_y.axes.autoscale_view()
            self.canvas_y.draw()

            self.canvas_z.axes.relim()
            self.canvas_z.axes.autoscale_view()
            self.canvas_z.draw()

            self.canvas_roll.axes.relim()
            self.canvas_roll.axes.autoscale_view()
            self.canvas_roll.draw()

            self.canvas_pitch.axes.relim()
            self.canvas_pitch.axes.autoscale_view()
            self.canvas_pitch.draw()

            self.canvas_yaw.axes.relim()
            self.canvas_yaw.axes.autoscale_view()
            self.canvas_yaw.draw()
        except Exception as e:
            self.ros_node.get_logger().error(f"Error updating plots: {e}")

        


    def on_button_1_click(self):
        """Function called when button 1 is clicked."""
        self.ros_node.publish_message("Button 1 Clicked")
        self.ui.label.setText(f"gz: {self.ros_node.data1}")

    def on_button_2_click(self):
        """Function called when button 2 is clicked."""
        self.ros_node.publish_message("Button 2 Clicked")
        self.ui.label.setText(f"gz: {self.ros_node.data2}")


def ros_spin_thread(node):
    """Run the ROS2 spin loop in a separate thread."""
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Error in ROS spin thread: {e}")


def main():
    """Main function to set up ROS2 node and PyQt5 application."""
    rclpy.init()  # Initialize ROS2
    app = QApplication(sys.argv)  # Initialize the PyQt application

    # Create the ROS 2 node
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
