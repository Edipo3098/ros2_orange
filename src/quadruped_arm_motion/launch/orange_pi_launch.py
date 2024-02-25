from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mpu_pub',
            namespace='',
            executable='mpu_publisher'
        ),
        Node(
            package='quadruped_arm_motion',
            namespace='',
            executable='motor_subscriber'
        )
    ])