import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Mpu, COGframe
from time import time
from scipy.integrate import cumtrapz
import numpy as np
from filterpy.kalman import KalmanFilter
class IMUKalmanFilter:
    def __init__(self, dt):
        self.kf = KalmanFilter(dim_x=9, dim_z=6)  # State vector has 9 dimensions, measurements have 6
        self.dt = dt
        
        # State transition matrix (F)
        self.kf.F = np.eye(9)
        self.kf.F[0, 3] = self.dt  # x += vx * dt
        self.kf.F[1, 4] = self.dt  # y += vy * dt
        self.kf.F[2, 5] = self.dt  # z += vz * dt
        
        # Measurement matrix (H)
        self.kf.H = np.zeros((6, 9))
        self.kf.H[0, 3] = 1  # acx -> vx
        self.kf.H[1, 4] = 1  # acy -> vy
        self.kf.H[2, 5] = 1  # acz -> vz
        self.kf.H[3, 6] = 1  # gx -> roll
        self.kf.H[4, 7] = 1  # gy -> pitch
        self.kf.H[5, 8] = 1  # gz -> yaw
        
        # Process noise covariance matrix (Q)
        self.kf.Q = np.diag([1e-5, 1e-5, 1e-5, 1e-3, 1e-3, 1e-3, 1e-5, 1e-5, 1e-5])
        
        # Measurement noise covariance matrix (R)
        self.kf.R = np.diag([0.5, 0.5, 0.5, 0.1, 0.1, 0.1])
        
        # Covariance matrix (P)
        self.kf.P = np.eye(9) * 0.01  # Low uncertainty in initial position and velocity
    
    def predict(self):
        self.kf.predict()
    
    def update(self, measurements):
        self.kf.update(measurements)
    
    def change_dt(self, dt):
        self.dt = dt
        self.kf.F[0, 3] = dt
        self.kf.F[1, 4] = dt
        self.kf.F[2, 5] = dt
    def get_state(self):
        return self.kf.x[:3], self.kf.x[3:6], self.kf.x[6:9]  # Position, velocity, orientation


class COGEstimationNode(Node):
    def __init__(self):
        super().__init__('cog_estimation_node')
        self.sub_mpu_1 = self.create_subscription(Mpu, 'mpu_data', self.mpu_callback_1, 10)
        self.sub_mpu_2 = self.create_subscription(Mpu, 'mpu_data_2', self.mpu_callback_2, 10)
        self.mpu_data_1 = None
        self.mpu_data_2 = None

        # Initialize Kalman filter
        self.kalman_filter = IMUKalmanFilter(dt=0.01)

    def mpu_callback_1(self, msg):
        self.mpu_data_1 = msg
        self.process_estimation()

    def mpu_callback_2(self, msg):
        self.mpu_data_2 = msg
        self.process_estimation()

    def process_estimation(self):
        if self.mpu_data_1 and self.mpu_data_2:
            # Average sensor data from both IMUs
            avg_acx = (self.mpu_data_1.acx + self.mpu_data_2.acx) / 2
            avg_acy = (self.mpu_data_1.acy + self.mpu_data_2.acy) / 2
            avg_acz = (self.mpu_data_1.acz + self.mpu_data_2.acz) / 2
            avg_gx = (self.mpu_data_1.gx + self.mpu_data_2.gx) / 2
            avg_gy = (self.mpu_data_1.gy + self.mpu_data_2.gy) / 2
            avg_gz = (self.mpu_data_1.gz + self.mpu_data_2.gz) / 2

            # Create measurement array
            measurements = np.array([avg_acx, avg_acy, avg_acz, avg_gx, avg_gy, avg_gz])

            # Use Kalman filter to estimate the state
            self.kalman_filter.predict()
            self.kalman_filter.update(measurements)
            position, velocity, orientation = self.kalman_filter.get_state()

            # Publish estimated COG frame
            msg = COGframe()
            msg.pos_x = float(position[0])
            msg.pos_y = float(position[1])
            msg.pos_z = float(position[2])
            msg.roll = float(orientation[0])
            msg.pitch = float(orientation[1])
            msg.yaw = float(orientation[2])
            self.get_logger().info(f"COG Estimated Position 0: {float(position[0])},  {float(position[1])}, {float(position[2])}")
            self.get_logger().info(f"COG Estimated orientation 0: {float(orientation[0])},  {float(orientation[1])}, {float(orientation[2])}")
            

def main(args=None):
    rclpy.init(args=args)
    cog_calculate = COGEstimationNode()

    rclpy.spin(cog_calculate)
    # Destroy the node explicitly
    cog_calculate.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

