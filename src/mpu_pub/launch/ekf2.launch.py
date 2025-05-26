from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_robot = get_package_share_directory('mpu_pub')
    ekf_yaml = os.path.join(pkg_robot, 'config', 'ekf.yaml')
    
    with open(ekf_yaml, 'r') as f:
        controllers_files = f.read()
    return LaunchDescription([
        
        Node(
            package='mpu_pub',
            executable='imu_bridge',
            name='imu_bridge'
        ),
         Node(
            package='mpu_pub',
            executable='data_calibrated_mpu',
            name='imu_bridge'
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter',
            parameters=[ekf_yaml]
        ),
    ])
