import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Mpu, COGframe
from time import time
import numpy as np
from filterpy.kalman import ExtendedKalmanFilter
from ahrs.filters import Madgwick  # Use Madgwick from ahrs package
from collections import deque
"""
Fusion sensor increase measuremetes to 12
6 for each sensor
pip install madgwick_py

"""



from filterpy.kalman import ExtendedKalmanFilter
import numpy as np

class IMUFusionEKF:
    def __init__(self, dt):
        self.dt = dt
        self.ekf = ExtendedKalmanFilter(dim_x=9, dim_z=3)  # 9 state variables, 3 fused measurements (linear acceleration)
        
        # Initial state vector (position, velocity, orientation)
        self.ekf.x = np.zeros(9)  # [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, roll, pitch, yaw]

        # State covariance matrix (P)
        self.ekf.P = np.eye(9) * 1  # Initial uncertainty

        # Process noise covariance matrix (Q)
        self.ekf.Q = np.diag([1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2, 1e-3, 1e-3, 1e-3])

        # Measurement noise covariance matrix (R)
        self.ekf.R = np.diag([1e-2, 1e-2, 1e-2])  # Noise for fused accelerations

        # Madgwick filter instance
        self.madgwick_filter = Madgwick(frequency=1/dt)

        # Initial quaternion for Madgwick filter
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])

    def state_transition_function(self, x, u):
        """
        State transition function for EKF.
        - x: state vector [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, roll, pitch, yaw]
        - u: fused accelerations [acx_fused, acy_fused, acz_fused]
        """
        dt = self.dt
        # Predict new position and velocity based on acceleration
        pos = x[:3] + x[3:6] * dt + 0.5 * u[:3] * dt**2  # Position update
        vel = x[3:6] + u[:3] * dt  # Velocity update based on acceleration

        return np.hstack((pos, vel))

    def predict(self, u):
        """
        Use filterpy's built-in EKF prediction function.
        - u: fused accelerations [acx_fused, acy_fused, acz_fused]
        """
        # Use the state transition function to predict new position and velocity
        self.ekf.x[:6] = self.state_transition_function(self.ekf.x, u)
        
        # Update covariance matrix with state transition
        F = self.calculate_jacobian(self.ekf.x)  # Jacobian matrix for state transition
        self.ekf.P = F @ self.ekf.P @ F.T + self.ekf.Q
    def measurement_function(self, x):
        """
        Measurement function that maps the state to the expected measurement.
        In this case, we're predicting accelerations (which are related to velocity).
        """
        # Return the velocity components from the state as expected measurements (acceleration)
        return np.array([x[3], x[4], x[5]])  # Velocity (vx, vy, vz) are proxies for acceleration
    def measurement_jacobian(self, x):
        """
        Jacobian of the measurement function (H).
        This maps the velocity components in the state vector to the measured accelerations.
        """
        H = np.zeros((3, 9))
        H[0, 3] = H[1, 4] = H[2, 5] = 1  # Map velocity to acceleration (3x9 matrix)
        return H
    def update(self, z_imu1, z_imu2, orientation_madgwick):
        """
        Update the EKF with fused accelerations and Madgwick orientation.
        - z_imu1: IMU1 accelerations [acx, acy, acz]
        - z_imu2: IMU2 accelerations [acx, acy, acz]
        - orientation_madgwick: [roll, pitch, yaw] from the Madgwick filter
        """
        # Fuse the accelerations from both IMUs (average or weighted)
        accel_fused = 0.5 * (z_imu1[:3] + z_imu2[:3])

        # Update the state vector's orientation directly from Madgwick
        self.ekf.x[6:9] = orientation_madgwick  # Update roll, pitch, yaw from Madgwick

        # Use filterpy's update function with the fused acceleration data
        self.ekf.update(z=accel_fused, HJacobian=self.measurement_jacobian, Hx=self.measurement_function)

    def calculate_jacobian(self, x):
        """
        Calculate the Jacobian matrix for the state transition function.
        """
        dt = self.dt
        F = np.eye(9)
        F[0, 3] = F[1, 4] = F[2, 5] = dt  # Partial derivatives for position w.r.t. velocity

        return F

    def change_dt(self, dt):
        self.dt = dt

    def get_state(self):
        """
        Return position, velocity, and orientation.
        Orientation comes from the Madgwick filter.
        """
        return self.ekf.x[:3], self.ekf.x[3:6], self.ekf.x[6:9]  # Position, velocity, orientation

class CalCOGFrame(Node):

    def __init__(self):
        super().__init__('cog_calc4')
        self.subscription_mpu = self.create_subscription(Mpu, 'mpu_data_1', self.listener_callback, 10)
        self.subscription_mpu2 = self.create_subscription(Mpu, 'mpu_data_2', self.listener_callback2, 10)
        
        self.publishKalmanFrame = self.create_publisher(COGframe, 'kalman_cog_frame_3', 10)
        self.publishTrapezFrame = self.create_publisher(COGframe, 'trapez_cog_frame_3', 10)

        self.alpha = 0.2  # Lower alpha = more smoothing, higher alpha = faster response
         # Initial values for filtered accelerometer data
        self.filtered_acc = {'acx': 0.0, 'acy': 0.0, 'acz': 0.0}
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # Initial quaternion representing no rotation
        # Initialize the measurement buffers for sliding window (store last N measurements)
        self.window_size = 100  # Number of recent measurements to consider
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
        # Kalman filter for fusing data from both MPUs
        self.kf = IMUFusionEKF(dt=0.02)
        # Madgwick filter initialization
        self.madgwick_filter = Madgwick(frequency=50.0,gain=0.033)  # Adjust sample period as needed
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
    def low_pass_filter(self, axis, raw_data):
            """
            Apply an exponential moving average (EMA) low-pass filter to accelerometer data.
            axis: 'acx', 'acy', or 'acz'
            raw_data: new raw accelerometer reading
            """
            # Update the filtered value using EMA formula
            self.filtered_acc[axis] = self.alpha * raw_data + (1 - self.alpha) * self.filtered_acc[axis]
            
            # Return the filtered value
            return self.filtered_acc[axis]
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

    def compensate_gravity(self,accel, roll, pitch):
        """
        Compensate for the gravitational component in the accelerometer reading
        using the roll and pitch from the Madgwick filter.

        accel: 3D vector of accelerometer data [ax, ay, az] (in m/s²)
        roll: roll angle in radians
        pitch: pitch angle in radians
        """
        # Gravity vector in the sensor frame
        g = np.array([0, 0, 9.81])  # Gravity in m/s²

        # Calculate the rotation matrix from the roll and pitch
        # Note: Yaw isn't needed for gravity compensation
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll), np.cos(roll)]])  # Rotation matrix for roll
        
        R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])  # Rotation matrix for pitch
        
        # Rotate gravity into the sensor frame
        g_sensor = R_x @ R_y @ g

        # Subtract the gravity vector from the accelerometer readings
        accel_compensated = accel - g_sensor

        return accel_compensated
    def process_fusion(self):
        if self.mpu1_data is None or self.mpu2_data is None:
            return  # Wait until data from both MPUs is available

        current_time = time()
        dt = current_time - self.prev_time
        if dt < 1e-6:
            dt = 1e-6  # Set a minimum time step threshold
         # Apply low-pass filter to the raw accelerometer data

        filtered_acx = self.low_pass_filter('acx', self.mpu1_data.acx)
        filtered_acy = self.low_pass_filter('acy', self.mpu1_data.acy)
        filtered_acz = self.low_pass_filter('acz', self.mpu1_data.acz)
        filtered_acx2 = self.low_pass_filter('acx', self.mpu2_data.acx)
        filtered_acy2 = self.low_pass_filter('acy', self.mpu2_data.acy)
        filtered_acz2 = self.low_pass_filter('acz', self.mpu2_data.acz)
        self.kf.change_dt(dt)
        self.prev_time = current_time  # Update previous time
        # Add new measurement data to the buffers
        self.add_measurement_to_buffers(self.mpu1_data,self.mpu2_data)
        # Update the measurement noise covariance matrix based on recent data
        self.update_measurement_noise()
        # Create the measurement vector with data from both IMUs
        
        # In process_fusion:
        
        alpha = 0.7  # Weight for IMU1, 1 - alpha for IMU2
        
        # Weighted average for gyroscope data
        gyroscope_data = np.array([
            alpha * self.mpu1_data.gx + (1 - alpha) * self.mpu2_data.gx,
            alpha * self.mpu1_data.gy + (1 - alpha) * self.mpu2_data.gy,
            alpha * self.mpu1_data.gz + (1 - alpha) * self.mpu2_data.gz
        ]) # already in rad/s

        # Weighted average for accelerometer data
        accelerometer_data = np.array([
            alpha * self.mpu1_data.acx + (1 - alpha) * self.mpu2_data.acx,
            alpha * self.mpu1_data.acy + (1 - alpha) * self.mpu2_data.acy,
            alpha * self.mpu1_data.acz + (1 - alpha) * self.mpu2_data.acz
        ]) / 9.81  # Convert to G
        self.quaternion
        self.quaternion  = self.madgwick_filter.updateIMU(q=self.quaternion,gyr=gyroscope_data, acc=accelerometer_data)

       
        roll, pitch, yaw = self.quaternion_to_euler_angles(self.quaternion)
        # Compensate for gravity using the orientation from the Madgwick filter
        # Convert accelerometer readings to m/s² (if not already in m/s²)
        accel_imu1 = np.array([self.mpu1_data.acx, self.mpu1_data.acy, self.mpu1_data.acz]) * 1
        accel_imu2 = np.array([self.mpu2_data.acx, self.mpu2_data.acy, self.mpu2_data.acz]) * 1
        accel_imu1_comp = self.compensate_gravity(accel_imu1, roll, pitch)
        accel_imu2_comp = self.compensate_gravity(accel_imu2, roll, pitch)

        # Fused measurement vector for EKF (acceleration from both IMUs)
        z_imu1 = np.array([self.mpu1_data.acx, self.mpu1_data.acy, self.mpu1_data.acz])
        z_imu2 = np.array([self.mpu2_data.acx, self.mpu2_data.acy, self.mpu2_data.acz])

            # Fuse the accelerations (average)
        u_fused = 0.5 * (z_imu1 + z_imu2)

        # EKF Prediction step with fused acceleration
        self.kf.predict(u_fused)

        # EKF Update step with fused measurements and Madgwick orientation Z
        self.kf.update(z_imu1, z_imu1, [roll, pitch, yaw])

        # Retrieve filtered state (position, velocity)
        pos, vel, orient = self.kf.get_state()

        # Retrieve filtered state (position, velocity)
        pos, vel, orient = self.kf.get_state()
        # Publish the Kalman filter output
        msg = COGframe()
        msg.pos_x, msg.pos_y, msg.pos_z = float(pos[0]), float(pos[1]), float(pos[2])
        msg.roll, msg.pitch, msg.yaw = float(roll), float(pitch), float(yaw)
        self.publishKalmanFrame.publish(msg)
        self.get_logger().info(f"Kalman ACC 1  2(vx vy vz): {float(accel_imu1[0])}, {float(accel_imu1[1])}, {float(accel_imu1[2])}")
        self.get_logger().info(f"Kalman acc 2 (vx vy vz): {float(accel_imu2[0])}, {float(accel_imu2[1])}, {float(accel_imu2[2])}")
        self.get_logger().info(f"Kalman acc -g 1  2(vx vy vz): {float(accel_imu1_comp[0])}, {float(accel_imu1_comp[1])}, {float(accel_imu1_comp[2])}")
        self.get_logger().info(f"Kalman acc -2  2 (vx vy vz): {float(accel_imu2_comp[0])}, {float(accel_imu2_comp[1])}, {float(accel_imu2_comp[2])}")

def main(args=None):
    rclpy.init(args=args)
    cog_calculate = CalCOGFrame()
    rclpy.spin(cog_calculate)
    cog_calculate.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
