from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QWidget, QStackedWidget,QVBoxLayout,QSizePolicy
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor,QPalette
from PyQt5.QtCore import QTimer
import numpy as np
class MyMplCanvas(FigureCanvas):
    """A Matplotlib canvas that can be embedded into a PyQt5 application."""
    def __init__(self, parent=None, width=8, height=10, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = None
        self.is_3d = False
        self.create_axes(is_3d=False)  # Default is 2D
        super(MyMplCanvas, self).__init__(self.fig)
        # Set explicit size for the canvas to make it larger in the PyQt5 layout
        self.setMinimumSize(width * 10, height * 10)  # Set minimum size based on width and height
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)


    def create_axes(self, is_3d=False):
        """Create or switch axes to either 2D or 3D."""
        self.fig.clear()  # Clear the current figure

        if is_3d:
            self.axes = self.fig.add_subplot(111, projection='3d')  # Create a 3D subplot
        else:
            self.axes = self.fig.add_subplot(111)  # Create a 2D subplot
        self.is_3d = is_3d

    def plot_circles(self,footPose,cogPose,quadMoving,legMoving,title):
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
        self.axes.relim()  # Recompute the limits based on current artists.
        self.axes.autoscale_view()  # Automatically scale the view to include all data.
        self.axes.set_title(title)
        self.axes.set_xlabel('X-axis')
        self.axes.set_ylabel('Y-axis')

        # Set axis limits and aspect ratio
        
        self.axes.set_aspect('auto')  # Allow dynamic scaling of the plot axes
        self.axes.grid(True)  # Enable grid for better visibility
        self.axes.tick_params(axis='x', which='both', bottom=True, top=True, labelbottom=True)  # Ensure tick labels and ticks are shown



        # Re-draw the plot
        self.draw()
    def plot_axes(self, base,yaw,ef, title):
        """Plot 3D coordinate axes based on the given position and orientation."""

        # Clear the axes
        self.axes.clear()

        self.plot_axi([0,0,0,0,0,0])
        
        self.plot_axi(ef)

        # Set plot limits, labels, and title
        self.axes.set_xlim(-3, 3)
        self.axes.set_ylim(-3, 3)
        self.axes.set_zlim(-3, 3)
        self.axes.set_title(title)
        self.axes.set_xlabel('X-axis')
        self.axes.set_ylabel('Y-axis')
        self.axes.set_zlabel('Z-axis')

        # Set aspect ratio and enable grid
        
        self.axes.grid(True)  # Enable grid for better visualization

        

        # Re-draw the plot to reflect updates
        self.draw()
    def plot_circle(self,footPose,title):
        """Plot five circles at specified positions."""
        # Define circle centers
        centers = footPose
        
        radius = 0.1  # Circle radius

        # Clear the axes
        self.axes.clear()
        self.axes.cla()
        
        
       # Plot each circle using matplotlib's Circle patch
        
        circle = Circle((centers[0], centers[1]), radius, color='red', fill=True)  # Create a circle using Circle from patches
        self.axes.add_patch(circle)
        # If there are multiple centers, connect them with a line
        
        self.axes.set_xlim(-1, 3)
        self.axes.set_ylim(-1, 3)
        self.axes.autoscale_view()  # Automatically scale the view to include all data.
        self.axes.set_title(title)
        self.axes.set_xlabel('X-axis')
        self.axes.set_ylabel('Y-axis')

        # Set axis limits and aspect ratio
        
        self.axes.set_aspect('auto')  # Allow dynamic scaling of the plot axes
        self.axes.grid(True)  # Enable grid for better visibility
        self.axes.tick_params(axis='x', which='both', bottom=True, top=True, labelbottom=True)  # Ensure tick labels and ticks are shown



        # Re-draw the plot
        self.draw()
    def plot_axes(self, base,yaw,ef, title):
        """Plot 3D coordinate axes based on the given position and orientation."""

        # Clear the axes
        self.axes.clear()

        self.plot_axi([0,0,0,0,0,0])
        
        self.plot_axi(ef)

        # Set plot limits, labels, and title
        self.axes.set_xlim(-3, 3)
        self.axes.set_ylim(-3, 3)
        self.axes.set_zlim(-3, 3)
        self.axes.set_title(title)
        self.axes.set_xlabel('X-axis')
        self.axes.set_ylabel('Y-axis')
        self.axes.set_zlabel('Z-axis')

        # Set aspect ratio and enable grid
        
        self.axes.grid(True)  # Enable grid for better visualization

        

        # Re-draw the plot to reflect updates
        self.draw()
    def plot_axi(self,pose):
        # Extract translation (position) and rotation (orientation) from the pose
        x, y, z = pose[:3]
        roll, pitch, yaw = pose[3:6]

        # Length of the axes for visualization
        axis_length = 0.5

        # Define rotation matrices for roll, pitch, yaw
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])

        R_y = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])

        R_z = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])

        # Combined rotation matrix: R = Rz * Ry * Rx
        R = R_z @ R_y @ R_x

        # Define origin of the coordinate system
        origin = np.array([x, y, z])

        # Define the end points for each axis
        x_axis = origin + R @ np.array([axis_length, 0, 0])
        y_axis = origin + R @ np.array([0, axis_length, 0])
        z_axis = origin + R @ np.array([0, 0, axis_length])

        # Plot the origin
        self.axes.scatter(*origin, color='black', s=50)

        # Plot the X, Y, and Z axes from the origin
        self.axes.quiver(*origin, *(x_axis - origin), color='red', arrow_length_ratio=0.01, label='X-axis')
        self.axes.quiver(*origin, *(y_axis - origin), color='green', arrow_length_ratio=0.01, label='Y-axis')
        self.axes.quiver(*origin, *(z_axis - origin), color='blue', arrow_length_ratio=0.01, label='Z-axis')