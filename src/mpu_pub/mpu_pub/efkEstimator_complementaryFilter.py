import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Mpu, COGframe
from time import time
import numpy as np
from filterpy.kalman import ExtendedKalmanFilter
from ahrs.filters import Madgwick  # Use Madgwick from ahrs package
from collections import deque
"""
Fusion sensor increase measuremetes to 6
Accx Accy AccZ
The fuses is being implemented in 
update that update uses both measurements
G

"""



from filterpy.kalman import ExtendedKalmanFilter
import numpy as np

class IMUFusionEKF:
    def __init__(self, dt):
        self.dt = dt
        self.ekf = ExtendedKalmanFilter(dim_x=9, dim_z=9)  # 9 state variables, 3 fused measurements (linear acceleration)
        
        # Initial state vector (position, velocity, orientation)
        self.ekf.x = np.zeros(9)  # [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, roll, pitch, yaw]

        # State covariance matrix (P)
        self.ekf.P = np.eye(9) * 1  # Initial uncertainty
        self.residuals_window = deque(maxlen=100)  # Store last 100 residuals
        # Process noise covariance matrix (Q)
        self.mul = 1000
        self.mulR = 0.001
        self.mulQ = 0.0001
        self.ekf.Q = np.diag([1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2, 1e-3, 1e-3, 1e-3])*self.mulR   # Noise for position, velocity, orientation

        # Measurement noise covariance matrix (R)
        self.ekf.R = np.diag([1e-2, 1e-2, 1e-2,1e-2, 1e-2, 1e-2,1e-2, 1e-2, 1e-2]) *self.mulQ  # Noise for fused accelerations

        

        # Initial quaternion for Madgwick filter
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])

        # COmplementary filter
        self.complementary_alpha = 0.98  # Giving more weight to gyroscope data
        # Store previous acceleration (initialized to zero)
        self.prev_acceleration = np.zeros(3)
        self.residuals_window = deque(maxlen=100)  # Store last 100 residuals


    def state_transition_function(self, x, u):
        """
        State transition function for EKF using trapezoidal integration.
        - x: state vector [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, roll, pitch, yaw]
        - u: fused accelerations [acx_fused, acy_fused, acz_fused]
        """
        dt = self.dt
        # Update velocity using trapezoidal integration: v_new = v_old + 0.5 * (a_prev + a_new) * dt
        current_acceleration = u[:3]
        prev_velocity = x[3:6]
        
        new_velocity = prev_velocity + 0.5 * (self.prev_acceleration + current_acceleration) * dt
        
        # Update position using trapezoidal integration: p_new = p_old + 0.5 * (v_old + v_new) * dt
        prev_position = x[:3]
        new_position = prev_position + 0.5 * (prev_velocity + new_velocity) * dt

        # Store current acceleration for the next iteration
        self.prev_acceleration = current_acceleration

        # Keep orientation unchanged in this step
        orientation = x[6:9]

        # Return the updated state
        return np.hstack((new_position, new_velocity, orientation))

    def predict(self, u):
        """
        Use filterpy's built-in EKF prediction function with the trapezoidal integration state transition.
        - u: fused accelerations [acx_fused, acy_fused, acz_fused]
        """
        # Use the modified state transition function to predict new position and velocity
        new_state = self.state_transition_function(self.ekf.x, u)

        # Update only the position and velocity in the state vector (the first 6 elements)
        self.ekf.x[:6] = new_state[:6]

        # Orientation remains unchanged from the prediction step
        # Update covariance matrix with state transition
        F = self.calculate_jacobian(self.ekf.x)  # Jacobian matrix for state transition
        self.ekf.P = F @ self.ekf.P @ F.T + self.ekf.Q

        # Optionally update Q based on state prediction error
        # self.update_Q(self.ekf.x)

    def measurement_function(self, x):
        """
        Measurement function that maps the state to the expected measurement.
        In this case, we're predicting accelerations (which are related to velocity).
        """
        # Predicted accelerations for IMU 1 and IMU 2 (using velocity components)
        acc_imu1 = np.array([x[3], x[4], x[5]])  # From velocity components
        acc_imu2 = np.array([x[3], x[4], x[5]])  # Assuming similar dynamics for IMU 2
        
        # Orientation from the state (roll, pitch, yaw)
        orientation = x[6:9]  # [roll, pitch, yaw]
        
        # Return a combined prediction of accelerations and orientation
        return np.hstack((acc_imu1, acc_imu2, orientation))
    def measurement_jacobian(self, x):
        """
        Jacobian of the measurement function (H).
        This maps the velocity components in the state vector to the measured accelerations
        and orientation components to the measured orientation.
        """
        H = np.zeros((9, 9))
        # Map velocity to acceleration (first 6 elements are for accelerations from IMU1 and IMU2)
        H[0, 3] = H[1, 4] = H[2, 5] = 1  # IMU1 acceleration from velocity
        H[3, 3] = H[4, 4] = H[5, 5] = 1  # IMU2 acceleration from velocity
        
        # Map orientation (roll, pitch, yaw)
        H[6, 6] = H[7, 7] = H[8, 8] = 1  # Roll, pitch, yaw from the state
        
        return H
    def update(self, z_imu1, z_imu2, orientation_madgwick):
        """
        Update the EKF with fused accelerations and Madgwick orientation.
        - z_imu1: IMU1 accelerations [acx, acy, acz]
        - z_imu2: IMU2 accelerations [acx, acy, acz]
        - orientation_madgwick: [roll, pitch, yaw] from the Madgwick filter
        """
        # Combine both IMUs' accelerations and Madgwick orientation into a single 9-dimensional vector
        measurements = np.hstack((z_imu1[:3], z_imu2[:3], orientation_madgwick))  # Shape (9,)
        
        # Compute predicted measurements from the current state
        Hx = self.measurement_function(self.ekf.x)  # Predicted measurement also of shape (9,)

        # Compute the residual (innovation) between the actual and predicted measurement
        residual = self.compute_residual(measurements, Hx)
        
        # Store the residual for adaptive noise estimation
        self.residuals_window.append(residual)
        # Update R dynamically based on residuals
        #self.update_R(list(self.residuals_window))

        # Perform the EKF update step
        try:
            self.ekf.update(z=measurements, HJacobian=self.measurement_jacobian, Hx=self.measurement_function)
        except np.linalg.LinAlgError:
            print("Singular matrix error, adding regularization")
            # Regularize P if there's an issue
            self.ekf.P += np.eye(9) * 1e-6

        # Regularize P after the update to avoid singularity issues in future updates
        self.ekf.P += np.eye(9) * 1e-6
        

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
    def compute_residual(self, z, Hx):
            """
            Compute the residual (innovation) between the actual measurement z 
            and the predicted measurement Hx.
            - z: actual measurement (from sensors)
            - Hx: predicted measurement (from the measurement function)
            
            print(f"Measurement z: {z.shape}")
            print(f"Measurement Hx: {Hx.shape}")
            """
            return z - Hx
    def update_R(self, residuals,alpha=0.1):
        """
        Dynamically update the measurement noise covariance R 
        based on the variance of the residuals.
        - residuals: list of past residuals to calculate variance
        """
        # Calculate the variance of the residuals for each sensor
        variance = np.var(residuals, axis=0)
            
            # Update the measurement noise covariance matrix R
        # Smooth the update using Exponential Moving Average (EMA)
        self.ekf.R = (1 - alpha) * self.ekf.R + alpha * np.diag(variance) * self.mul


    def update_Q(self, state_predictions,beta = 0.1):
        """
        Update the process noise covariance matrix Q based on the variance of prediction errors.
        - state_predictions: list of past prediction errors (state differences)
        """
        variance = np.var(state_predictions, axis=0)
        #self.ekf.Q = np.diag(variance) * self.mul  # Adjust the process noise accordingly
        # Calculate the variance of the prediction error for each state variable
        #prediction_error_variance = np.var(state_prediction_error, axis=0)
        variance = np.var(state_predictions, axis=0)
        print(f"Variance: {variance}")
        if variance.ndim > 1:
            variance = variance.flatten()
        # Smooth the update of Q
        self.ekf.Q = ((1 - beta) * self.ekf.Q + beta * np.diag(12*variance) )* self.mul
    def update_noise_covariances(self, accel_data, accel_data2, gyro_data, predicted_state):
        """
        Dynamically adjust the process noise covariance (Q) and measurement noise covariance (R) 
        based on the variance of accelerometer, gyroscope data, and the predicted state (position, velocity, orientation).
        
        predicted_state: The predicted state vector [p_x, p_y, p_z, v_x, v_y, v_z, roll, pitch, yaw].
        """
        # Convert to a numpy array
        
        accel_variance = np.array(list(accel_data.values()))
        accel_variance2 = np.array(list(accel_data2.values()))
        gyro_variance =np.array(list(gyro_data.values()))
        
        stateVariance = np.array(list(predicted_state.values()))
        # Variance from the predicted state (position, velocity, orientation)
        position_variance = stateVariance[0:3]
        velocity_variance =  stateVariance[3:6]
        orientation_variance = stateVariance[6:9]
        

        # Update process noise covariance Q dynamically based on state prediction variances
        self.ekf.Q = np.diag([position_variance[0], position_variance[1], position_variance[2],
                            velocity_variance[0], velocity_variance[1], velocity_variance[2],
                            orientation_variance[0], orientation_variance[1], orientation_variance[2]])*self.mulQ
        

        # Update measurement noise covariance R dynamically based on accelerometer and gyroscope variances
        self.ekf.R = np.diag([accel_variance[0], accel_variance[1], accel_variance[2],
                            accel_variance2[0], accel_variance2[1], accel_variance2[2],
                            gyro_variance[0], gyro_variance[1], gyro_variance[2]])*self.mulR
        
    def complementary_filter(self, accel1, accel2, gyro1, gyro2, dt, prev_orientation):
        """
        Implements a complementary filter for two MPUs to fuse accelerometer and gyroscope data.
        :param accel1: Accelerometer data from MPU1 [acc_x1, acc_y1, acc_z1]
        :param accel2: Accelerometer data from MPU2 [acc_x2, acc_y2, acc_z2]
        :param gyro1: Gyroscope data from MPU1 [gyro_x1, gyro_y1, gyro_z1]
        :param gyro2: Gyroscope data from MPU2 [gyro_x2, gyro_y2, gyro_z2]
        :param dt: time step
        :param prev_orientation: previous orientation [roll, pitch, yaw]
        :return: fused [roll, pitch, yaw]
        """
        # Fuse accelerometer and gyroscope data from both MPUs (e.g., using a weighted average)
        alpha = 0.5  # Assuming equal weight for both MPUs

        accel_fused = alpha * accel1 + (1 - alpha) * accel2
        gyro_fused = alpha * gyro1 + (1 - alpha) * gyro2

        # Gyroscope integration for short-term orientation (high-pass)
        roll_gyro = prev_orientation[0] + gyro_fused[0] * dt
        pitch_gyro = prev_orientation[1] + gyro_fused[1] * dt
        yaw_gyro = prev_orientation[2] + gyro_fused[2] * dt

        # Accelerometer for long-term orientation (low-pass)
        roll_acc = np.arctan2(accel_fused[1], np.sqrt(accel_fused[0] ** 2 + accel_fused[2] ** 2))
        pitch_acc = np.arctan2(-accel_fused[0], np.sqrt(accel_fused[1] ** 2 + accel_fused[2] ** 2))
        yaw_acc = prev_orientation[2]  # Cannot determine yaw from accelerometer

        # Complementary filter to combine the two estimates
        roll = self.complementary_alpha * roll_gyro + (1 - self.complementary_alpha) * roll_acc
        pitch = self.complementary_alpha * pitch_gyro + (1 - self.complementary_alpha) * pitch_acc
        yaw = yaw_gyro  # Yaw primarily from gyroscope

        return np.array([roll, pitch, yaw])

class CalCOGFrame(Node):

    def __init__(self):
        super().__init__('efk_estimator_2')
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
            'roll': deque(maxlen=self.window_size),
            'pitch': deque(maxlen=self.window_size),
            'yaw': deque(maxlen=self.window_size),
        }
        self.acc_buffers2 = {
            'acx': deque(maxlen=self.window_size),
            'acy': deque(maxlen=self.window_size),
            'acz': deque(maxlen=self.window_size),
        }
        self.statePredictions = {
            'posX' : deque(maxlen=self.window_size),
            'posY' : deque(maxlen=self.window_size),
            'posZ' : deque(maxlen=self.window_size),
            'velX' : deque(maxlen=self.window_size),
            'velY' : deque(maxlen=self.window_size),
            'velZ' : deque(maxlen=self.window_size),
            'roll' : deque(maxlen=self.window_size),
            'pitch' : deque(maxlen=self.window_size),
            'yaw' : deque(maxlen=self.window_size),
        }
        # Kalman filter for fusing data from both MPUs
        self.frec = 2000
        self.kf = IMUFusionEKF(dt=1/self.frec )
        # Madgwick filter initialization
        self.madgwick_filter = Madgwick(frequency=self.frec ,gain=0.0033)  # Adjust sample period as needed
        """
        If your IMU data is noisy, a lower beta value may help reduce jitter, though you will need to balance this with the slower data rate.
        """
        # Buffers to hold the last measurements from each MPU
        self.mpu1_data = None
        self.mpu2_data = None
        self.calibrated = False

        self.prev_time = time()
        self.prevOrientation  = np.array([0.0, 0.0, 0.0])

        self.pos = np.array([0.0, 0.0, 0.0])
        self.orient  = np.array([0.0, 0.0, 0.0])
        self.timer2 = self.create_timer(1, self.timerCallback2)


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
        self.acc_variance = {
            axis: np.var(self.acc_buffers[axis]) if len(self.acc_buffers[axis]) > 1 else 1.0
            for axis in ['acx', 'acy', 'acz']
        }

        # Compute variance for gyroscope
        self.gyro_variance = {
            axis: np.var(self.gyro_buffers[axis]) if len(self.gyro_buffers[axis]) > 1 else 1.0
            for axis in ['roll', 'pitch', 'yaw']
        }
         # Compute variance for accelerometer
        self.acc_variance2 = {
            axis: np.var(self.acc_buffers2[axis]) if len(self.acc_buffers2[axis]) > 1 else 1.0
            for axis in ['acx', 'acy', 'acz']
        }

        self.state_variance = {
            axis: np.var(self.statePredictions[axis]) if len(self.statePredictions[axis]) > 1 else 1.0
            for axis in ['posX', 'posY', 'posZ','velX','velY','velZ','roll','pitch','yaw']
        }
        
 
    def add_measurement_to_buffers(self, imu_data,orientation,statesEstimatos):
        """
        Add new IMU data to the sliding window buffers for noise calculation.
        """
        self.acc_buffers['acx'].append(imu_data.acx)
        self.acc_buffers['acy'].append(imu_data.acy)
        self.acc_buffers['acz'].append(imu_data.acz)
        self.gyro_buffers['roll'].append(orientation[0])
        self.gyro_buffers['pitch'].append(orientation[1])
        self.gyro_buffers['yaw'].append(orientation[2])

        self.acc_buffers2['acx'].append(imu_data.acx2)
        self.acc_buffers2['acz'].append(imu_data.acz2)
        self.acc_buffers2['acy'].append(imu_data.acy2)


        self.statePredictions['posX'].append(statesEstimatos[0])
        self.statePredictions['posY'].append(statesEstimatos[1])
        self.statePredictions['posZ'].append(statesEstimatos[2])
        self.statePredictions['velX'].append(statesEstimatos[3])
        self.statePredictions['velY'].append(statesEstimatos[4])
        self.statePredictions['velZ'].append(statesEstimatos[5])
        self.statePredictions['roll'].append(statesEstimatos[6])
        self.statePredictions['pitch'].append(statesEstimatos[7])
        self.statePredictions['yaw'].append(statesEstimatos[8])
        
    
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
        g = np.array([0, 0, -9.81])  # Gravity in m/s²

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
    def compensate_gravity_with_quaternion(self, accel, q):
        """
        Compensate for the gravitational component in the accelerometer reading
        using the quaternion from the Madgwick filter to avoid singularities.
        
        Parameters:
        - accel: 3D vector of accelerometer data [ax, ay, az] (in m/s²)
        - q: quaternion as [w, x, y, z]

        Returns:
        - accel_compensated: gravity-compensated acceleration vector
        """
        # Quaternion components
        w, x, y, z = q

        # Gravity vector in the global frame (assuming Z points upwards)
        gravity = np.array([0, 0, -9.81])

        # Convert quaternion to rotation matrix to rotate gravity vector
        # This is the rotation matrix derived from the quaternion
        R = np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),     1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
            [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x**2 + y**2)]
        ])

        # Rotate the gravity vector into the sensor frame
        gravity_sensor_frame = R @ gravity

        # Subtract the gravity vector from the accelerometer readings
        accel_compensated = accel - gravity_sensor_frame

        return accel_compensated
    
   

    def process_fusion(self):
        

        current_time = time()
        dt = current_time - self.prev_time
        if dt < 1e-6:
            dt = 1e-6  # Set a minimum time step threshold
         # Apply low-pass filter to the raw accelerometer data

        self.mpu1_data.acy = abs(self.mpu1_data.acy*0.001)
        self.mpu1_data.acy2 = abs(self.mpu1_data.acy2*0.00001)
        filtered_acx = self.low_pass_filter('acx', self.mpu1_data.acx)
        filtered_acy = self.low_pass_filter('acy', self.mpu1_data.acy)
        filtered_acz = self.low_pass_filter('acz', self.mpu1_data.acz)
        filtered_acx2 = self.low_pass_filter('acx', self.mpu1_data.acx2)
        filtered_acy2 = self.low_pass_filter('acy', self.mpu1_data.acy2)
        filtered_acz2 = self.low_pass_filter('acz', self.mpu1_data.acz2)
        self.kf.change_dt(dt)
        self.prev_time = current_time  # Update previous time
        
        # Create the measurement vector with data from both IMUs
        
        # In process_fusion:
        
        alpha = 0.6  # Weight for IMU1, 1 - alpha for IMU2
        
        # Weighted average for gyroscope data in
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
        ])*9.81
        gyroscope_data_filtered = np.array([
            alpha * self.mpu1_data.gx + (1 - alpha) * self.mpu1_data.gx2,
            alpha * self.mpu1_data.gy + (1 - alpha) * self.mpu1_data.gy2,
            alpha * self.mpu1_data.gz + (1 - alpha) * self.mpu1_data.gz2
        ]) # already in rad/s

        # Weighted average for accelerometer data
        accelerometer_data_filtered = np.array([
            alpha * filtered_acx + (1 - alpha) * filtered_acx2,
            alpha * filtered_acy + (1 - alpha) * filtered_acy2,
            alpha * filtered_acz + (1 - alpha) * filtered_acz2
        ])*9.81   # Convert to m/s²
        self.quaternion
        gyro_imu1 = np.array([self.mpu1_data.gx, self.mpu1_data.gy, self.mpu1_data.gz])
        gyro_imu2 = np.array([self.mpu1_data.gx2, self.mpu1_data.gy2, self.mpu1_data.gz2])
        # Use complementary filter to calculate orientation
        
        self.quaternion  = self.madgwick_filter.updateIMU(q=self.quaternion,gyr=gyroscope_data_filtered, acc=accelerometer_data_filtered)

        
        roll, pitch, yaw = self.quaternion_to_euler_angles(self.quaternion)  # in rads
        orientation = np.array([roll, pitch, yaw])
        # Compensate for gravity using the orientation from the Madgwick filter
        # Convert accelerometer readings to m/s² (if not already in m/s²)
        
        accel_imu1_raw = np.array([self.mpu1_data.acx, self.mpu1_data.acy, self.mpu1_data.acz]) 
        accel_imu2_raw = np.array([self.mpu1_data.acx2, self.mpu1_data.acy2, self.mpu1_data.acz2]) 
        accel_imu1 = np.array([self.mpu1_data.acx, self.mpu1_data.acy, self.mpu1_data.acz]) 
        accel_imu2 = np.array([self.mpu1_data.acx2, self.mpu1_data.acy2, self.mpu1_data.acz2]) 
        accel_imu1filt = np.array([filtered_acx, filtered_acy, filtered_acz]) 
        accel_imu2filt = np.array([filtered_acx2, filtered_acy2, filtered_acz2]) 
        accel_imu1_comp = self.compensate_gravity_with_quaternion(accel_imu1, self.quaternion)
        accel_imu2_comp = self.compensate_gravity_with_quaternion(accel_imu2, self.quaternion)
        accel_imu1_comp_filt = self.compensate_gravity_with_quaternion(accel_imu1filt, self.quaternion)
        accel_imu2_comp_filt = self.compensate_gravity_with_quaternion(accel_imu2filt, self.quaternion)
        # Fused measurement vector for EKF (acceleration from both IMUs)
        #orientation = self.kf.complementary_filter(accel_imu1, accel_imu2, gyro_imu1, gyro_imu2, dt, self.prevOrientation )
        #self.prevOrientation = orientation
        z_imu1 = accel_imu1filt
        z_imu2 = accel_imu1filt
            # Fuse the accelerations (average)
        u_fused = alpha*z_imu1 + (1-alpha)*z_imu2
        
        # EKF Prediction step with fused acceleration
        # Add new measurement data to the buffers

        
        self.kf.predict(u_fused)

        # EKF Update step with fused measurements and Madgwick orientation Z
        self.kf.update(z_imu1, z_imu2, orientation)

        # Retrieve filtered state (position, velocity)
        pos, vel, orient = self.kf.get_state()
        self.add_measurement_to_buffers(self.mpu1_data,orientation,self.kf.ekf.x)
        self.update_measurement_noise()
        self.kf.update_noise_covariances(self.acc_variance, self.acc_variance2, self.gyro_variance, self.state_variance)
        
        # Publish the Kalman filter output
        msg = COGframe()
        msg.pos_x, msg.pos_y, msg.pos_z = float(pos[0]), float(pos[1]), float(pos[2])
        msg.roll, msg.pitch, msg.yaw = float(orient[0]), float(orient[1]), float(orient[2])
        self.pos = pos
        self.orient = orient
        self.publishKalmanFrame.publish(msg)
        if ( not self.calibrated ):
            if (accel_imu1_raw[0] < 0.1 and accel_imu1_raw[1] < 0.1 and accel_imu1_raw[2] < 0.1 and accel_imu2_raw[0] < 0.1 and accel_imu2_raw[1] < 0.1 and accel_imu2_raw[2] < 0.1):
                self.calibrated = True
                self.calibration_time = time()
                self.get_logger().info("Calibration  done")
            
        else:
            self.publishKalmanFrame.publish(msg)

     
        
        """
        self.get_logger().info(f"MPU 1 Filtered: {float(filtered_acx)}, {float(filtered_acy)},{float(filtered_acz)}")
        self.get_logger().info(f"MPU 2 Filtered: {float(filtered_acx2)}, {float(filtered_acy2)},{float(filtered_acz2)}")
        self.get_logger().info(f"MPU 1  compensate G: {float(accel_imu1_comp[0])}, {float(accel_imu1_comp[1])}, {float(accel_imu1_comp[2])}")
        self.get_logger().info(f"MPU 2  compensate G: {float(accel_imu2_comp[0])}, {float(accel_imu2_comp[1])}, {float(accel_imu2_comp[2])}")
        self.get_logger().info(f"MPU 1  compensate G filt: {float(accel_imu1_comp_filt[0])}, {float(accel_imu1_comp_filt[1])}, {float(accel_imu1_comp_filt[2])}")
        self.get_logger().info(f"MPU 2  compensate G filt: {float(accel_imu2_comp_filt[0])}, {float(accel_imu2_comp_filt[1])}, {float(accel_imu2_comp_filt[2])}")
        """
    def timerCallback2(self):
        self.get_logger().info("Timer callback 2")
        self.get_logger().info(f"Pos X Y Z (meter) dt : {float(self.pos[0])}, {float(self.pos[1])}, {float(self.pos[2])}, {1/self.kf.dt}")
        self.get_logger().info(f"Roll pitch yaw  : {float(self.orient[0])}, {float(self.orient[1])}, {float(self.orient[2])}")
        
def main(args=None):
    rclpy.init(args=args)
    efkEstimator = CalCOGFrame()
    rclpy.spin(efkEstimator)
    efkEstimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
