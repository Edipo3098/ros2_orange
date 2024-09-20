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





class IMUFusionEKF:
    def __init__(self, dt):
        self.dt = dt
        
        # Now, use dim_z=12 for 12 fused measurements (6 from each IMU)
        self.ekf = ExtendedKalmanFilter(dim_x=12, dim_z=12)

        # Initial state vector (position, velocity, orientation, gravity)
        self.ekf.x = np.zeros(12)  # [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, roll, pitch, yaw, gravity_x, gravity_y, gravity_z]

        # State covariance matrix (P)
        self.ekf.P = np.eye(12) * 1  # Updated to 12x12 for the new state vector

        # Process noise covariance matrix (Q) - also 12x12
        self.mul  = 10
        self.residuals_window = deque(maxlen=100)  # Store last 100 residuals
        self.ekf.Q = np.diag([1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2, 1e-3, 1e-3, 1e-3, 1e-4, 1e-4, 1e-4])*self.mul 

        # Measurement noise covariance matrix (R) - 12x12 for 12 measurements
        self.ekf.R = np.diag([1e-2] * 12 )

        

        # Initial quaternion for Madgwick filter
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])

    def state_transition_function(self, x, u):
        """
        State transition function for EKF.
        - x: state vector [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, roll, pitch, yaw, gravity_x, gravity_y, gravity_z]
        - u: fused input [acx_fused, acy_fused, acz_fused, gyr_x, gyr_y, gyr_z]
        """
        dt = self.dt
        pos = x[:3] + x[3:6] * dt + 0.5 * u[:3] * dt**2  # Position update using acceleration
        vel = x[3:6] + u[:3] * dt  # Velocity update using acceleration
        
        # Update orientation (roll, pitch, yaw) using angular velocity (gyroscope)
        roll = x[6] + u[3] * dt  # Angular velocity affects roll
        pitch = x[7] + u[4] * dt  # Angular velocity affects pitch
        yaw = x[8] + u[5] * dt  # Angular velocity affects yaw
        return np.hstack((pos, vel, roll, pitch, yaw, x[9:12]))  # Gravity components remain unchanged
    # Return the updated state
   

    def predict(self, u):
        """
        Predict step for EKF, using acceleration and angular velocity (gyroscope).
        - u: fused input [acx_fused, acy_fused, acz_fused, gyr_x, gyr_y, gyr_z]
        """
        self.ekf.x = self.state_transition_function(self.ekf.x, u)
        
        # Calculate Jacobian matrix (F) for the state transition
        F = self.calculate_jacobian(self.ekf.x)
        
        # Update the state covariance matrix (P)
        self.ekf.P = F @ self.ekf.P @ F.T + self.ekf.Q
    def measurement_function(self, x):
    
        """
        Measurement function that maps the state to the expected measurement.
        Predict accelerations and gyroscope data based on velocity and orientation.
        - x: state vector [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, roll, pitch, yaw, grav_x, grav_y, grav_z]
        """
        # Accelerometer measurements with gravity compensation
        acc_imu1 = np.array([x[3] + x[9], x[4] + x[10], x[5] + x[11]])  # Accelerations from IMU1
        acc_imu2 = np.array([x[3] + x[9], x[4] + x[10], x[5] + x[11]])  # Assuming similar dynamics for IMU2
        
        # Gyroscope readings (angular velocities are roll, pitch, yaw)
        gyro_imu1 = np.array([x[6], x[7], x[8]])  # IMU1 orientation rates (roll, pitch, yaw)
        gyro_imu2 = np.array([x[6], x[7], x[8]])  # Assuming similar dynamics for IMU2

        # Create the full measurement vector
        full_measurement = np.hstack((acc_imu1, gyro_imu1, acc_imu2, gyro_imu2))
        
        # Log the size and values of the full measurement vector
        print(f"Measurement function output size: {full_measurement.shape}")
        print(f"Measurement function output content: {full_measurement}")
        
        # Return both accelerometer and gyroscope data for IMU1 and IMU2
        return full_measurement
    def measurement_jacobian(self, x):
        """
        Jacobian of the measurement function (H). Maps the state vector to 12 measurements.
        Handles both accelerometer and gyroscope measurements for IMU1 and IMU2.
        """
        H = np.zeros((12, 12))
        
        # Accelerometer mapping to velocity and gravity
        H[0, 3] = H[1, 4] = H[2, 5] = 1  # IMU1 accelerometer to velocity
        H[0, 9] = H[1, 10] = H[2, 11] = 1  # IMU1 accelerometer to gravity components
        
        H[6, 3] = H[7, 4] = H[8, 5] = 1  # IMU2 accelerometer to velocity
        H[6, 9] = H[7, 10] = H[8, 11] = 1  # IMU2 accelerometer to gravity components
        
        # Gyroscope mapping to orientation (roll, pitch, yaw)
        H[3, 6] = H[4, 7] = H[5, 8] = 1  # IMU1 gyroscope to orientation
        H[9, 6] = H[10, 7] = H[11, 8] = 1  # IMU2 gyroscope to orientation (assumed similar)
        
        return H
    def update(self, z_imu1, z_imu2, orientation_madgwick):
        """
        Update step for EKF, using fused accelerations and gyroscope data from both IMUs and Madgwick orientation.
        - z_imu1: IMU1 accelerations [acx, acy, acz]
        - z_imu2: IMU2 accelerations [acx, acy, acz]
        - orientation_madgwick: [roll, pitch, yaw] from Madgwick filter
        """
        # Combine both IMUs' measurements into a single 12-dimensional vector
        # Compute predicted measurements from the current state
        print(f"State covariance matrix (P) before update size: {self.ekf.P.shape}")
        Hx = self.measurement_function(self.ekf.x)
        # Log the size and values of the predicted measurement Hx
        print(f"Predicted measurement (Hx) size: {Hx.shape}")
        print(f"Predicted measurement (Hx) content: {Hx}")
        measurements = np.hstack((z_imu1, orientation_madgwick, z_imu2, orientation_madgwick))
        print(f"Measurements vector size: {measurements.shape}")
        print(f"Measurements vector content: {measurements}")
        # Compute residual (innovation)
        residual = self.compute_residual(measurements, Hx)
        # Store the residual for adaptive noise estimation
        self.residuals_window.append(residual)
        # Periodically update measurement noise covariance R based on residual variance
        if len(self.residuals_window) > 12:  # Ensure enough residuals for variance calculation
            self.update_R(self.residuals_window)
        # Update orientation directly from Madgwick filter
        self.ekf.x[6:9] = orientation_madgwick

        
        
        # EKF Update step
        try:
            self.ekf.update(z=measurements, HJacobian=self.measurement_jacobian, Hx=self.measurement_function)
        except np.linalg.LinAlgError:
            print("Singular matrix error, adding regularization")
            # Regularize P if there's an issue
            self.ekf.P += np.eye(12) * 1e-6

        # Regularize P after the update to avoid singularity issues in future updates
         # Log the state covariance matrix after regularization
        print(f"State covariance matrix (P) after regularization size: {self.ekf.P.shape}")



    def calculate_jacobian(self, x):
        """
        Calculate the Jacobian matrix for the state transition function.
        """
        dt = self.dt
        F = np.eye(12)
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
    def compute_residual(self, z, Hx):
        """
        Compute the residual (innovation) between the actual measurement z 
        and the predicted measurement Hx.
        - z: actual measurement (from sensors)
        - Hx: predicted measurement (from the measurement function)
        """
        return z - Hx
    def update_R(self, residuals):
        """
        Dynamically update the measurement noise covariance R 
        based on the variance of the residuals.
        - residuals: list of past residuals to calculate variance
        """
        # Calculate the variance of the residuals for each sensor
        variance = np.var(residuals, axis=0)
        
        # Update the measurement noise covariance matrix R
        self.ekf.R = np.diag(variance) * self.mul  # self.kf.mul is your multiplier
        # Log the size and values of R
        print(f"Measurement noise covariance (R) size: {self.ekf.R.shape}")
        print(f"Measurement noise covariance (R) content: {self.ekf.R}")
    def update_Q(self, state_predictions):
        """
        Update the process noise covariance matrix Q based on the variance of prediction errors.
        - state_predictions: list of past prediction errors (state differences)
        """
        variance = np.var(state_predictions, axis=0)
        self.ekf.Q = np.diag(variance) * self.mul  # Adjust the process noise accordingly



class CalCOGFrame(Node):

    def __init__(self):
        super().__init__('cog_calc3')
        self.subscription_mpu = self.create_subscription(Mpu, 'mpu_data_1', self.listener_callback, 10)
        
        
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
        self.frecuency = 100
        self.kf = IMUFusionEKF(dt=1/self.frecuency)  # Initialize the EKF with the sample period
        # Madgwick filter initialization
        self.madgwick_filter = Madgwick(frequency=self.frecuency,gain=0.033)  # Adjust sample period as needed
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
        """
        self.kf.ekf.R = np.diag([
            acc_variance['acx'], acc_variance['acy'], acc_variance['acz'],
            gyro_variance['gx'], gyro_variance['gy'], gyro_variance['gz'],
            acc_variance2['acx'], acc_variance2['acy'], acc_variance2['acz'],
            gyro_variance2['gx'], gyro_variance2['gy'], gyro_variance2['gz']
        ])*self.kf.mul 
        """
    def add_measurement_to_buffers(self, imu_data):
        """
        Add new IMU data to the sliding window buffers for noise calculation.
        """
        self.acc_buffers['acx'].append(imu_data.acx)
        self.acc_buffers['acy'].append(imu_data.acy)
        self.acc_buffers['acz'].append(imu_data.acz)
        self.gyro_buffers['gx'].append(imu_data.gx)
        self.gyro_buffers['gy'].append(imu_data.gy)
        self.gyro_buffers['gz'].append(imu_data.gz)

        self.acc_buffers2['acx'].append(imu_data.acx2)
        self.acc_buffers2['acz'].append(imu_data.acz2)
        self.acc_buffers2['acy'].append(imu_data.acy2)
        self.gyro_buffers2['gx'].append(imu_data.gx2)
        self.gyro_buffers2['gy'].append(imu_data.gy2)
        self.gyro_buffers2['gz'].append(imu_data.gz2)
    
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
        

        current_time = time()
        dt = current_time - self.prev_time
        if dt < 1e-6:
            dt = 1e-6  # Set a minimum time step threshold
         # Apply low-pass filter to the raw accelerometer data

        filtered_acx = self.low_pass_filter('acx', self.mpu1_data.acx)
        filtered_acy = self.low_pass_filter('acy', self.mpu1_data.acy)
        filtered_acz = self.low_pass_filter('acz', self.mpu1_data.acz)
        filtered_acx2 = self.low_pass_filter('acx', self.mpu1_data.acx2)
        filtered_acy2 = self.low_pass_filter('acy', self.mpu1_data.acy2)
        filtered_acz2 = self.low_pass_filter('acz', self.mpu1_data.acz2)
        self.kf.change_dt(dt)
        self.prev_time = current_time  # Update previous time
        # Add new measurement data to the buffers
        self.add_measurement_to_buffers(self.mpu1_data)
        # Update the measurement noise covariance matrix based on recent data
        self.update_measurement_noise()
        # Create the measurement vector with data from both IMUs
        
        # In process_fusion:
        
        alpha = 0.7  # Weight for IMU1, 1 - alpha for IMU2
        
        # Weighted average for gyroscope data
        gyroscope_data = np.array([
            alpha * self.mpu1_data.gx + (1 - alpha) * self.mpu1_data.gx2,
            alpha * self.mpu1_data.gy + (1 - alpha) * self.mpu1_data.gy2,
            alpha * self.mpu1_data.gz + (1 - alpha) * self.mpu1_data.gz2
        ]) # already in rad/s

        # Weighted average for accelerometer data
        accelerometer_data = np.array([
            alpha * self.mpu1_data.acx + (1 - alpha) * self.mpu1_data.acx2,
            alpha * self.mpu1_data.acy + (1 - alpha) * self.mpu1_data.acy2,
            alpha * self.mpu1_data.acz + (1 - alpha) * self.mpu1_data.acz2
        ])   # Convert to G
        self.quaternion
        self.quaternion  = self.madgwick_filter.updateIMU(q=self.quaternion,gyr=gyroscope_data, acc=accelerometer_data)

       
        roll, pitch, yaw = self.quaternion_to_euler_angles(self.quaternion)
        # Compensate for gravity using the orientation from the Madgwick filter
        # Convert accelerometer readings to m/s² (if not already in m/s²)
        accel_imu1_raw = np.array([self.mpu1_data.acx, self.mpu1_data.acy, self.mpu1_data.acz]) 
        accel_imu2_raw = np.array([self.mpu1_data.acx2, self.mpu1_data.acy2, self.mpu1_data.acz2]) 
        accel_imu1 = np.array([self.mpu1_data.acx, self.mpu1_data.acy, self.mpu1_data.acz]) * 9.81
        accel_imu2 = np.array([self.mpu1_data.acx2, self.mpu1_data.acy2, self.mpu1_data.acz2]) * 9.81

        accel_imu1filt = np.array([filtered_acx, filtered_acy, filtered_acz]) * 9.81
        accel_imu2filt = np.array([filtered_acx2, filtered_acy2, filtered_acz2]) * 9.81

        accel_imu1_comp = self.compensate_gravity(accel_imu1, roll, pitch)
        accel_imu2_comp = self.compensate_gravity(accel_imu2, roll, pitch)

        accel_imu1_comp_filt = self.compensate_gravity(accel_imu1filt, roll, pitch)
        accel_imu2_comp_filt = self.compensate_gravity(accel_imu2filt, roll, pitch)
        # Fused measurement vector for EKF (acceleration from both IMUs)
        z_imu1 = np.array([self.mpu1_data.acx, self.mpu1_data.acy, self.mpu1_data.acz])
        z_imu2 = np.array([self.mpu1_data.acx2, self.mpu1_data.acy2, self.mpu1_data.acz2])

            # Fuse the accelerations (average)
         # Fuse the accelerations (average of both IMUs)
        u1_acc = accel_imu1_raw
        u2_acc = accel_imu2_raw

        u1_gyro = np.array([self.mpu1_data.gx, self.mpu1_data.gy, self.mpu1_data.gz])*np.pi/180 # Convert to rad/s
        u2_gyro = np.array([self.mpu1_data.gx2, self.mpu1_data.gy2, self.mpu1_data.gz2])*np.pi/180 # Convert to rad/s

        u1 = np.concatenate((accel_imu1_raw, u2_acc))
        u2 = np.concatenate((u2_acc, u2_gyro))
        multiplier = 0.2
        u_fused = multiplier * (u1 )+  (1-multiplier)* u2
        
        # EKF Prediction step with fused acceleration
        self.kf.predict(u_fused)

        # EKF Update step with fused measurements and Madgwick orientation
        self.kf.update(u1_acc, u2_acc, [roll, pitch, yaw])

        # Retrieve the filtered state (position, velocity, orientation)
        pos, vel, orient = self.kf.get_state()

        
        # Publish the Kalman filter output
        msg = COGframe()
        msg.pos_x, msg.pos_y, msg.pos_z = float(pos[0]), float(pos[1]), float(pos[2])
        msg.roll, msg.pitch, msg.yaw = float(roll), float(pitch), float(yaw)
        self.publishKalmanFrame.publish(msg)
        self.get_logger().info(f"MPU 1 raw: {float(accel_imu1_raw[0])}, {float(accel_imu1_raw[1])}, {float(accel_imu1_raw[2])}")
        self.get_logger().info(f"MPU 2 raw:  {float(accel_imu2_raw[0])}, {float(accel_imu2_raw[1])}, {float(accel_imu2_raw[2])}")
        self.get_logger().info(f"MPU 1: {float(accel_imu1[0])}, {float(accel_imu1[1])}, {float(accel_imu1[2])}")
        self.get_logger().info(f"MPU 2:  {float(accel_imu2[0])}, {float(accel_imu2[1])}, {float(accel_imu2[2])}")
        self.get_logger().info(f"MPU 1 Filtered: {float(accel_imu1filt[0])}, {float(accel_imu1filt[1])},{float(accel_imu1filt[2])}")
        self.get_logger().info(f"MPU 2 Filtered: {float(accel_imu2filt[0])}, {float(accel_imu2filt[1])},{float(accel_imu2filt[2])}")
        self.get_logger().info(f"Pos X Y Z: {float(pos[0])}, {float(pos[1])}, {float(pos[2])}")


        """
        self.get_logger().info(f"MPU 1 Filtered: {float(filtered_acx)}, {float(filtered_acy)},{float(filtered_acz)}")
        self.get_logger().info(f"MPU 2 Filtered: {float(filtered_acx2)}, {float(filtered_acy2)},{float(filtered_acz2)}")
        self.get_logger().info(f"MPU 1  compensate G: {float(accel_imu1_comp[0])}, {float(accel_imu1_comp[1])}, {float(accel_imu1_comp[2])}")
        self.get_logger().info(f"MPU 2  compensate G: {float(accel_imu2_comp[0])}, {float(accel_imu2_comp[1])}, {float(accel_imu2_comp[2])}")
        self.get_logger().info(f"MPU 1  compensate G filt: {float(accel_imu1_comp_filt[0])}, {float(accel_imu1_comp_filt[1])}, {float(accel_imu1_comp_filt[2])}")
        self.get_logger().info(f"MPU 2  compensate G filt: {float(accel_imu2_comp_filt[0])}, {float(accel_imu2_comp_filt[1])}, {float(accel_imu2_comp_filt[2])}")
        """
def main(args=None):
    rclpy.init(args=args)
    cog_calculate = CalCOGFrame()
    rclpy.spin(cog_calculate)
    cog_calculate.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
