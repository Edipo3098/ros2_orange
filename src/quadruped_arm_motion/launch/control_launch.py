from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mpu_pub',
            namespace='',
            executable='mpu_subscriber',
        ),
        Node(
            package='quadruped_arm_motion',
            namespace='',
            executable='motor_control'
            
        )
    ])