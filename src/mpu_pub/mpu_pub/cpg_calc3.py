import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Mpu, COGframe
from time import time
import numpy as np
from filterpy.kalman import KalmanFilter
from madgwick_py import MadgwickAHRS
from collections import deque
"""
Fusion sensor increase measuremetes to 12
6 for each sensor
pip install madgwick_py

"""

class IMUKalmanFilter:
    def __init__(self, dt):
        self.kf = KalmanFilter(dim_x=9, dim_z=12)  # 9 state variables, 12 measurements (6 from each IMU)
        self.dt = dt

        # Gyroscope noise variance (º/s)^2
        self.gyro_rms_noise = 0.1  # º/s-rms
        self.gyro_variance = self.gyro_rms_noise ** 2  # Variance for gx, gy, gz
        self.gyro_variance2 = self.gyro_rms_noise ** 2  # Variance for gx, gy, gz
        # Accelerometer noise variance (m/s^2)^2
        self.acc_rms_noise = 0.008 * 9.81  # Convert from g to m/s^2
        self.acc_variance = self.acc_rms_noise ** 2  # Variance for acx, acy, acz
        self.acc_variance2 = self.acc_rms_noise ** 2  # Variance for acx, acy, acz
        self.kf.x = np.zeros(9)  # Initial state: assume system is stationary initially

        # State transition matrix (F)
        self.kf.F = np.eye(9)
        self.kf.F[0, 3] = self.dt  # x += vx * dt
        self.kf.F[1, 4] = self.dt  # y += vy * dt
        self.kf.F[2, 5] = self.dt  # z += vz * dt
        
        # Measurement matrix (H) for both IMUs
        self.kf.H = np.zeros((12, 9))
        # IMU 1
        self.kf.H[0, 3] = 1  # acx_1 -> vx
        self.kf.H[1, 4] = 1  # acy_1 -> vy
        self.kf.H[2, 5] = 1  # acz_1 -> vz
        self.kf.H[3, 6] = 1  # gx_1 -> roll
        self.kf.H[4, 7] = 1  # gy_1 -> pitch
        self.kf.H[5, 8] = 1  # gz_1 -> yaw
        # IMU 2
        self.kf.H[6, 3] = 1  # acx_2 -> vx
        self.kf.H[7, 4] = 1  # acy_2 -> vy
        self.kf.H[8, 5] = 1  # acz_2 -> vz
        self.kf.H[9, 6] = 1  # gx_2 -> roll
        self.kf.H[10, 7] = 1  # gy_2 -> pitch
        self.kf.H[11, 8] = 1  # gz_2 -> yaw
        
        # Process noise covariance matrix (Q)
        self.kf.Q = np.diag([1e-5, 1e-5, 1e-5, 1e-3, 1e-3, 1e-3, 1e-5, 1e-5, 1e-5])
        
        # Measurement noise covariance matrix (R)
        self.kf.R = np.diag([self.acc_variance, self.acc_variance, self.acc_variance, 
                             self.gyro_variance, self.gyro_variance, self.gyro_variance,
                             self.acc_variance, self.acc_variance, self.acc_variance, 
                             self.gyro_variance, self.gyro_variance, self.gyro_variance])
        
        # Covariance matrix (P)
        self.kf.P = np.eye(9) * 1  # Initial uncertainty

        # Initialize the measurement buffers for sliding window (store last N measurements)
        self.window_size = 50  # Number of recent measurements to consider
        self.acc_buffers = {
            'acx': deque(maxlen=self.window_size),
            'acy': deque(maxlen=self.window_size),
            'acz': deque(maxlen=self.window_size),
        }
        self.gyro_buffers = {
            'gx': deque(maxlen=self.window_size),
            'gy': deque(maxlen=self.window_size),
            'gz': deque(maxlen=self.window_size),
        }
        self.acc_buffers2 = {
            'acx': deque(maxlen=self.window_size),
            'acy': deque(maxlen=self.window_size),
            'acz': deque(maxlen=self.window_size),
        }
        self.gyro_buffers2 = {
            'gx': deque(maxlen=self.window_size),
            'gy': deque(maxlen=self.window_size),
            'gz': deque(maxlen=self.window_size),
        }
    def update_measurement_noise(self):
        """
        Calculate new measurement noise covariance matrix R based on sliding window variance.
        """
        # Compute variance for accelerometer
        acc_variance = {
            axis: np.var(self.acc_buffers[axis]) if len(self.acc_buffers[axis]) > 1 else 1.0
            for axis in ['acx', 'acy', 'acz']
        }

        # Compute variance for gyroscope
        gyro_variance = {
            axis: np.var(self.gyro_buffers[axis]) if len(self.gyro_buffers[axis]) > 1 else 1.0
            for axis in ['gx', 'gy', 'gz']
        }
         # Compute variance for accelerometer
        acc_variance2 = {
            axis: np.var(self.acc_buffers2[axis]) if len(self.acc_buffers2[axis]) > 1 else 1.0
            for axis in ['acx', 'acy', 'acz']
        }

        # Compute variance for gyroscope
        gyro_variance2 = {
            axis: np.var(self.gyro_buffers2[axis]) if len(self.gyro_buffers2[axis]) > 1 else 1.0
            for axis in ['gx', 'gy', 'gz']
        }
        # Update the measurement noise covariance matrix (R)
        self.kf.R = np.diag([
            acc_variance['acx'], acc_variance['acy'], acc_variance['acz'],
            gyro_variance['gx'], gyro_variance['gy'], gyro_variance['gz'],
            acc_variance2['acx'], acc_variance2['acy'], acc_variance2['acz'],
            gyro_variance2['gx'], gyro_variance2['gy'], gyro_variance2['gz']
        ])

    def add_measurement_to_buffers(self, imu_data,imu_data2):
        """
        Add new IMU data to the sliding window buffers for noise calculation.
        """
        self.acc_buffers['acx'].append(imu_data.acx)
        self.acc_buffers['acy'].append(imu_data.acy)
        self.acc_buffers['acz'].append(imu_data.acz)
        self.gyro_buffers['gx'].append(imu_data.gx)
        self.gyro_buffers['gy'].append(imu_data.gy)
        self.gyro_buffers['gz'].append(imu_data.gz)

        self.acc_buffers2['acx'].append(imu_data2.acx)
        self.acc_buffers2['acy'].append(imu_data2.acy)
        self.acc_buffers2['acz'].append(imu_data2.acz)
        self.gyro_buffers2['gx'].append(imu_data2.gx)
        self.gyro_buffers2['gy'].append(imu_data2.gy)
        self.gyro_buffers2['gz'].append(imu_data2.gz)

    def predict(self):
        self.kf.predict()
    
    def update(self, measurements):
        self.kf.update(measurements)
    
    def change_dt(self, dt):
        self.dt = dt
        self.kf.F[0, 3] = dt  # x += vx * dt
        self.kf.F[1, 4] = dt  # y += vy * dt
        self.kf.F[2, 5] = dt  # z += vz * dt
        
        # Adjust process noise covariance matrix (Q) based on dt
        self.kf.Q = np.diag([1e-5, 1e-5, 1e-5, 1e-3 * dt, 1e-3 * dt, 1e-3 * dt, 1e-5, 1e-5, 1e-5])

    
    def get_state(self):
        return self.kf.x[:3], self.kf.x[3:6], self.kf.x[6:9]  # Position, velocity, orientation

class CalCOGFrame(Node):

    def __init__(self):
        super().__init__('cog_calc3')
        self.subscription_mpu = self.create_subscription(Mpu, 'mpu_data', self.listener_callback, 10)
        self.subscription_mpu2 = self.create_subscription(Mpu, 'mpu_data_2', self.listener_callback2, 10)
        
        self.publishKalmanFrame = self.create_publisher(COGframe, 'kalman_cog_frame', 10)
        self.publishTrapezFrame = self.create_publisher(COGframe, 'trapez_cog_frame', 10)

        # Kalman filter for fusing data from both MPUs
        self.kf = IMUKalmanFilter(dt=0.02)
        # Madgwick filter initialization
        self.madgwick_filter = MadgwickAHRS(sampleperiod=0.02,beta = 0.1)  # Adjust sample period as needed
        """
        If your IMU data is noisy, a lower beta value may help reduce jitter, though you will need to balance this with the slower data rate.
        """
        # Buffers to hold the last measurements from each MPU
        self.mpu1_data = None
        self.mpu2_data = None

        self.prev_time = time()

    def listener_callback(self, msg):
        self.mpu1_data = msg
        self.process_fusion()
        
    def listener_callback2(self, msg):
        self.mpu2_data = msg
        self.process_fusion()

    def quaternion_to_euler_angles(self, q):
        """
        Convert a quaternion into Euler angles (roll, pitch, yaw)
        q: quaternion as [w, x, y, z]
        """
        w, x, y, z = q
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, 1.0)
        pitch = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)

        return roll, pitch, yaw


    def process_fusion(self):
        if self.mpu1_data is None or self.mpu2_data is None:
            return  # Wait until data from both MPUs is available

        current_time = time()
        dt = current_time - self.prev_time
        if dt < 1e-6:
            dt = 1e-6  # Set a minimum time step threshold

        self.kf.change_dt(dt)
        self.prev_time = current_time  # Update previous time
        # Add new measurement data to the buffers
        self.add_measurement_to_buffers(self.mpu1_data,self.mpu2_data)
        # Update the measurement noise covariance matrix based on recent data
        self.update_measurement_noise()
        # Create the measurement vector with data from both IMUs
        measurements = np.array([self.mpu1_data.acx, self.mpu1_data.acy, self.mpu1_data.acz,
                                 self.mpu1_data.gx, self.mpu1_data.gy, self.mpu1_data.gz,
                                 self.mpu2_data.acx, self.mpu2_data.acy, self.mpu2_data.acz,
                                 self.mpu2_data.gx, self.mpu2_data.gy, self.mpu2_data.gz])

        self.kf.predict()  # Prediction step based on the current state
        self.kf.update(measurements)  # Update Kalman filter with new measurements
        # Madgwick filter update (Orientation calculation)
        
        alpha = 0.7  # Weight for IMU1, 1 - alpha for IMU2

        # Weighted average for gyroscope data
        gyroscope_data = np.array([
            alpha * self.mpu1_data.gx + (1 - alpha) * self.mpu2_data.gx,
            alpha * self.mpu1_data.gy + (1 - alpha) * self.mpu2_data.gy,
            alpha * self.mpu1_data.gz + (1 - alpha) * self.mpu2_data.gz
        ]) * np.pi / 180  # Convert to rad/s

        # Weighted average for accelerometer data
        accelerometer_data = np.array([
            alpha * self.mpu1_data.acx + (1 - alpha) * self.mpu2_data.acx,
            alpha * self.mpu1_data.acy + (1 - alpha) * self.mpu2_data.acy,
            alpha * self.mpu1_data.acz + (1 - alpha) * self.mpu2_data.acz
        ]) / 9.81  # Convert to G
        # Inputs: gyroscope in rad/s and accelerometer in G (gravity)
        # Update Madgwick filter
        self.madgwick_filter.update_imu(gyroscope_data, accelerometer_data)

        # Retrieve quaternion from Madgwick filter
        quaternion = self.madgwick_filter.quaternion.q  # [w, x, y, z]

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        roll, pitch, yaw = self.quaternion_to_euler_angles(quaternion)

        # Retrieve filtered state (position, velocity, orientation)
        pos, vel, orient = self.kf.get_state()

        # Publish the Kalman filter output
        msg = COGframe()
        msg.pos_x, msg.pos_y, msg.pos_z = float(pos[0]), float(pos[1]), float(pos[2])
        msg.roll, msg.pitch, msg.yaw = float(roll), float(pitch), float(yaw)
        self.publishKalmanFrame.publish(msg)
        self.get_logger().info(f"Kalman pose (X, Y, Z): {msg.pos_x}, {msg.pos_y}, {msg.pos_z}")
        self.get_logger().info(f"Kalman orientation (roll pitch yaw): {msg.roll}, {msg.pitch}, {msg.yaw}")

def main(args=None):
    rclpy.init(args=args)
    cog_calculate = CalCOGFrame()
    rclpy.spin(cog_calculate)
    cog_calculate.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
