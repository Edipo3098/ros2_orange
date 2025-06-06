from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_file = PathJoinSubstitution(
        #[get_package_share_directory('quadruped_arm_motion'), 'urdf', 'quadruped_arm.urdf']
        [get_package_share_directory('quadruped_arm_motion'), 'urdf', 'quadruped_arm_robot_Control_effort.urdf']
        #[get_package_share_directory('quadruped_arm_motion'), 'urdf', 'quadruped_arm_robot_fixedbase.urdf']
    )
    
    # Resolve the URDF file path as a string
    urdf_file_resolved = urdf_file.perform(None)
    
    # Read the URDF file content
    with open(urdf_file_resolved, 'r') as infp:
        robot_description_content = infp.read()

    return LaunchDescription([
        
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_pub_mapd_world',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_pub_map_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_pub_odom_base',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'], #arguments=['0', '0', '0', '0', '0', '0', 'odom', 'chasis_link'],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
        
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),

        
    ])
