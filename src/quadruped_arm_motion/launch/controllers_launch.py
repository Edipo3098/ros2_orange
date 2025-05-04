from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare("quadruped_arm_motion")

    return LaunchDescription([
        # Spawner para broadcaster (estado de joints)
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen"
        ),

        # Espera 2 segundos y lanza controlador de esfuerzo
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["effort_hold_all"],
                    output="screen"
                )
            ]
        ),

        # Espera 5 segundos y lanza (no activa aún) el de trayectoria
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["position_trajectory_controller"],
                    output="screen"
                )
            ]
        )
    ])
