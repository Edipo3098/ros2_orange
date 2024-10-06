import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Locate the package and YAML file
   

    return LaunchDescription([
        Node(
            package='mpu_pub',
            namespace='',
            executable='mpu_publisher',
            output='screen',
        ),
        Node(
            package='mpu_pub',
            namespace='',
            executable='motor_subscriber',
            output='screen',
            
        )
    ])
