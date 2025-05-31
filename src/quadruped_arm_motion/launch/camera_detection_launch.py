from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_file = PathJoinSubstitution(
        #[get_package_share_directory('quadruped_arm_motion'), 'urdf', 'quadruped_arm.urdf']
        [get_package_share_directory('quadruped_arm_motion'), 'urdf', 'quadruped_arm_robot_control_real_effortTest_develope.urdf']
        #[get_package_share_directory('quadruped_arm_motion'), 'urdf', 'quadruped_arm_robot_fixedbase.urdf']
    )
    
    # Resolve the URDF file path as a string
    urdf_file_resolved = urdf_file.perform(None)
    
    # Read the URDF file content
    with open(urdf_file_resolved, 'r') as infp:
        robot_description_content = infp.read()
    tags_yaml = '/home/edipo/ros2_orange/src/apriltag_ros/cfg/tags_36h11.yaml'
    rviz_config_path = os.path.expanduser('~/.rviz2/quad.rviz')
    return LaunchDescription([
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d', rviz_config_path
            ],
            # Otras opciones (por ejemplo, nodename, remappings) se
            # pueden agregar aquí si lo necesitas.
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}],
        ),
        Node(
            package='Quad_control_gui',
            executable='control_robot_node',
            name='control_robot_node',
            output='screen',
            
        ),
        #Node(
        #    package='joint_state_publisher_gui',
        #    executable='joint_state_publisher_gui',
        #    name='joint_state_publisher_gui',
        #    output='screen',
        #),
        # transforma map → odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),

        # transforma odom → base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_odom_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen'
        ),
        # Ejemplo para tag 0
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_tag0',
            arguments=['2.146', '0.393', '-0.477', '3.14', '0.0', '-0.0', 'map', 'tag36h11:0_fixed'],
            output='screen'
        ),

        # Ejemplo para tag 1
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_tag1',
            arguments=['2.0796', '-0.3191', '-0.50', '3.14', '0.0', '-0.0', 'map',  'tag36h11:1_fixed'],
            output='screen'
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[
                {'video_device': '/dev/video0'},
                {'image_width': 1280},
                {'image_height': 720},
                {'framerate': 30.0},
                {'camera_info_url': 'file:///home/edipo/.ros/camera_info/default_cam_2.yaml'},
                {'frame_id': 'camera_link'}  ,
                {'pixel_format': 'yuyv'}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
          # corrección de distorsión
        Node(
            package='image_proc', executable='image_proc',
            name='image_proc', output='screen',
            remappings=[
                ('image', '/image_raw'),
                ('camera_info', 'camera_info'),
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_detector',
            output='screen',
            parameters=[
                {'family': '36h11'},
                {'size': 0.1},
                '/home/edipo/ros2_orange/src/apriltag_ros/cfg/tags_36h11.yaml',
                {'publish_tf': True},
                { 'publish_tag_detections_pose': True},
                {'camera_frame_id': 'camera_link'},
                
                
            ],
            remappings=[
                ('image', '/image_raw'),
                ('/camera_info', '/camera_info')
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
        
        Node(
            package='quadruped_arm_motion',
            executable='tag_pose_extractor',
            name='tag_pose_extractor',
            output='screen',
            arguments=['--ros-args', '--log-level', 'error']
        ),
        
        
    ])
