from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot = get_package_share_directory('quadruped_arm_motion')

    urdf_file = os.path.join(pkg_robot, 'urdf', 'quadruped_arm_robot_control_real_effortTest_develope.urdf')
    urdf_file1 = os.path.join(pkg_robot, 'urdf', 'quadruped_arm_robot_control_real_effort.urdf')
    
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    controllers_file = os.path.join(pkg_robot, 'config', 'controllers.yaml')
    
    with open(controllers_file, 'r') as f:
        controllers_files = f.read()

    
    return LaunchDescription([
        # 1. Lanzar Gazebo vacío
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            )
        ),

        # 2. Lanzar robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        
        
        
        # 2.1. Lanzar el controlador 
        # Nodo para cargar el ros2_control desde YAML
        #Node(
        #    package='controller_manager',
        #    executable='ros2_control_node',
        #    parameters=[
        #        {'robot_description': robot_description},
        #        controllers_files  # ¡Aquí sí va el YAML!
        #    ],
        #    output='screen'
        #),
        # 3. Spawnear el robot en Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'quadruped', '-topic', 'robot_description',
                       '-x', '0.0', '-y', '0.0', '-z', '0.375'  ],# <-- Añade posición inicial
            output='screen'
        ),

        # 4. Esperar 5 segundos antes de lanzar los controladores
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster'],
                    output='screen'
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_group_position_controller'],
                    output='screen'
                ),
                
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_group_effort_controller'],
                    output='screen'
                ),
                
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['trajectory_controller','--inactive'],
                    output='screen'
                ),
                
            ]
        ),

        # 5. (Opcional) Lanzar tu nodo dynamic_simulation
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='quadruped_arm_motion',
                    executable='dynamic_simulation',
                    name='dynamic_simulation',
                    output='screen'
                )
            ]
        )
    ])
