import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Mpu,COGframe
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import numpy as np
import math
DEG2RAD = math.pi/180.0
G2MS2   = 9.80665          # 1 g → 9.81 m/s²
CLAMP_DT = (1e-4, 0.05)    # 0.1 ms – 50 ms  (20–10000 Hz)
class ImuBridge(Node):
    def __init__(self):
        super().__init__('imu_bridge')
        # Suscríbete al mensaje combinado de tus dos MPU
        self.sub = self.create_subscription(
            Mpu, 'mpu_data_2', self.callback_mpu, 10)
        self.sub_odometry = self.create_subscription(Odometry, 'odometry/filtered', self.callback_odometry, 10)
        # Publicadores de los dos Imu separados
        self.pub0 = self.create_publisher(Imu, 'imu/primary', 10)
        self.pub1 = self.create_publisher(Imu, 'imu/secondary', 10)
        self.pub_COG  = self.create_publisher(COGframe, 'kalman_cog_frame_3', 10)
        
    def callback_mpu(self, msg: Mpu):
        # Timestamp común
        now = self.get_clock().now().to_msg()

        # Primer IMU
        imu0 = Imu()
        imu0.header = Header(stamp=now, frame_id='imu0_link')
        # Convertir g -> m/s² y °/s -> rad/s
        imu0.linear_acceleration.x = msg.acx * 9.81
        imu0.linear_acceleration.y = msg.acy * 9.81
        imu0.linear_acceleration.z = msg.acz * 9.81
        imu0.angular_velocity.x = msg.gx * np.pi/180.0
        imu0.angular_velocity.y = msg.gy * np.pi/180.0
        imu0.angular_velocity.z = msg.gz * np.pi/180.0

        # Segunda IMU
        imu1 = Imu()
        imu1.header = Header(stamp=now, frame_id='imu1_link')
        imu1.linear_acceleration.x = msg.acx2 * 9.81
        imu1.linear_acceleration.y = msg.acy2 * 9.81
        imu1.linear_acceleration.z = msg.acz2 * 9.81
        imu1.angular_velocity.x = msg.gx2 * np.pi/180.0
        imu1.angular_velocity.y = msg.gy2 * np.pi/180.0
        imu1.angular_velocity.z = msg.gz2 * np.pi/180.0

        # Publica ambos Imu
        self.pub0.publish(imu0)
        self.pub1.publish(imu1)
    def callback_odometry(self, msg: Odometry):
        # Timestamp común
        now = self.get_clock().now().to_msg()

        # COG
        cog = COGframe()
        #cog.header = Header(stamp=now, frame_id='imu0_link')
        cog.pos_x = msg.pose.pose.position.x
        cog.pos_y = msg.pose.pose.position.y
        cog.pos_z = msg.pose.pose.position.z
        cog.roll = msg.pose.pose.orientation.x
        cog.pitch = msg.pose.pose.orientation.y
        cog.yaw = msg.pose.pose.orientation.z
        self.pub_COG.publish(cog)
def main(args=None):
    rclpy.init(args=args)
    node = ImuBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
