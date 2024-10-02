import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Locate the package and YAML file
    package_dir = get_package_share_directory('mpu_pub')
    param_file = os.path.join(package_dir, 'config', 'ekf.yaml')

    return LaunchDescription([
        Node(
            package='mpu_pub',
            namespace='',
            executable='data_calibrated_mpu',
            output='screen',
        ),
        Node(
            package='mpu_pub',
            namespace='',
            executable='efk_estimator',
            output='screen',
            
        )
    ])
