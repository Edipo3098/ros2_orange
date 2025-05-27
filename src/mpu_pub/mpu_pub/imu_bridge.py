import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Mpu, COGframe
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np
import math

# Constantes
DEG2RAD = math.pi/180.0
G2MS2   = 9.80665

class ImuBridge(Node):
    def __init__(self):
        super().__init__('imu_bridge')
        # Suscripción al mensaje combinado de tus dos MPU
        self.sub = self.create_subscription(Mpu, 'mpu_data_2', self.callback_mpu, 10)
        # Publicador de zero-twist para ZUPT
        self.zero_odom_pub = self.create_publisher( Odometry, '/zero_twist', 10)
        # Publicadores de los dos Imu separados
        self.pub0 = self.create_publisher(Imu, 'imu/primary', 10)
        self.pub1 = self.create_publisher(Imu, 'imu/secondary', 10)
        # Publicador de COG
        self.pub_COG = self.create_publisher(COGframe, 'kalman_cog_frame_3', 10)

        # Umbrales para detectar «quietud»
        self.acc_threshold = 0.05    # m/s²
        self.gyro_threshold = 0.02   # rad/s

    def callback_mpu(self, msg: Mpu):
        now = self.get_clock().now().to_msg()

        # --- Construcción del mensaje IMU0 ---
        imu0 = Imu()
        imu0.header = Header(stamp=now, frame_id='imu1_link')
        ax0 = (msg.acx) * G2MS2
        ay0 = (msg.acy) * G2MS2
        az0 = (msg.acz) * G2MS2
        gx0 = (msg.gx) * DEG2RAD
        gy0 = (msg.gy) * DEG2RAD
        gz0 = (msg.gz) * DEG2RAD
        imu0.linear_acceleration.x = ax0
        imu0.linear_acceleration.y = ay0
        imu0.linear_acceleration.z = az0
        imu0.angular_velocity.x = gx0
        imu0.angular_velocity.y = gy0
        imu0.angular_velocity.z = gz0
        imu0.linear_acceleration_covariance = [0.1,0.0,0.0,  0.0,0.1,0.0,  0.0,0.0,0.1]
        imu0.angular_velocity_covariance    = [0.1,0.0,0.0,  0.0,0.1,0.0,  0.0,0.0,0.1]

        # --- Construcción del mensaje IMU1 ---
        imu1 = Imu()
        imu1.header = Header(stamp=now, frame_id='imu2_link')
        ax1 = (msg.acx2) * G2MS2
        ay1 = (msg.acy2) * G2MS2
        az1 = (msg.acz2) * G2MS2
        gx1 = (msg.gx2) * DEG2RAD
        gy1 = (msg.gy2) * DEG2RAD
        gz1 = (msg.gz2) * DEG2RAD
        imu1.linear_acceleration.x = ax1
        imu1.linear_acceleration.y = ay1
        imu1.linear_acceleration.z = az1
        imu1.angular_velocity.x = gx1
        imu1.angular_velocity.y = gy1
        imu1.angular_velocity.z = gz1
        imu1.linear_acceleration_covariance = [0.1,0.0,0.0,  0.0,0.1,0.0,  0.0,0.0,0.1]
        imu1.angular_velocity_covariance    = [0.1,0.0,0.0,  0.0,0.1,0.0,  0.0,0.0,0.1]

        # Publica IMUs
        self.pub0.publish(imu0)
        self.pub1.publish(imu1)

        # --- Zero-Velocity Update cuando ambas IMUs estén en quietud ---
        stationary0 = (
            abs(ax0) < self.acc_threshold and
            abs(ay0) < self.acc_threshold and
            abs(az0 ) < self.acc_threshold and
            abs(gx0) < self.gyro_threshold and
            abs(gy0) < self.gyro_threshold and
            abs(gz0) < self.gyro_threshold
        )
        stationary1 = (
            abs(ax1) < self.acc_threshold and
            abs(ay1) < self.acc_threshold and
            abs(az1 ) < self.acc_threshold and
            abs(gx1) < self.gyro_threshold and
            abs(gy1) < self.gyro_threshold and
            abs(gz1) < self.gyro_threshold
        )
        if stationary0 and stationary1:
            zero_odom = Odometry()
            zero_odom.header = zero_odom.header
            zero_odom.header.frame_id = 'base_link'
            zero_odom.child_frame_id = 'base_link'
            zero_odom.twist = zero_odom.twist
            # (podéis dejar pose a cero)
            # Añade aquí la línea de log
            # ---- Aquí defines explícitamente el twist y covarianza ----
            zero_odom.twist.twist.linear.x = 0.0
            zero_odom.twist.twist.linear.y = 0.0
            zero_odom.twist.twist.linear.z = 0.0
            zero_odom.twist.twist.angular.x = 0.0
            zero_odom.twist.twist.angular.y = 0.0
            zero_odom.twist.twist.angular.z = 0.0

            # Covarianza pequeña → alta confianza en ceros
            cov = [1e-9 if i in (0,7,14) else 0.0 for i in range(36)]
            zero_odom.twist.covariance = cov
            #self.get_logger().info('ZUPT detectado: publicando velocidad cero en /zero_twist')

            
            self.zero_odom_pub.publish(zero_odom)
            

    def callback_odometry(self, msg: Odometry):
        now = self.get_clock().now().to_msg()
        cog = COGframe()
        cog.pos_x = msg.pose.pose.position.x
        cog.pos_y = msg.pose.pose.position.y
        cog.pos_z = msg.pose.pose.position.z
        cog.roll  = msg.pose.pose.orientation.x
        cog.pitch = msg.pose.pose.orientation.y
        cog.yaw   = msg.pose.pose.orientation.z
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
