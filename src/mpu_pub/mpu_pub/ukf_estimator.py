import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Mpu, COGframe
from time import perf_counter
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

from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints

import numpy as np

class IMUFusionUKF:
    def __init__(self, dt):
        # Define the UKF parameters
        self.dt = dt
        self.points = MerweScaledSigmaPoints(9, alpha=0.1, beta=2.0, kappa=0.0)
        self.ukf =  UKF(dim_x=9, dim_z=9, dt=self.dt, fx=self.fx, hx=self.hx, points=self.points)
        # Initial state vector [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, roll, pitch, yaw]
        self.ukf.x = np.zeros(9)
        # Covariances (initial)
        self.ukf.P = np.eye(9) * 0.1  # State covariance matrix with low initial uncertainty

        self.ukf.Q = np.eye(9) * 0.01  # Process noise covariance matrix
        # Measurement noise covariance matrix
        # 3 Accelerometer measurements MPU 1 (x, y, z) 
        self.ukf.R = np.eye(9) * 0.1   # Measurement noise covariance matrix 3 Accele
        self.mul = 1000
        self.mulR = 1000
        self.mulQ = 0.00001
        self.complementary_alpha = 0.98  # Giving more weight to gyroscope data
        # Set the inverse function to use pseudo-inverse instead
        self.ukf.inv = np.linalg.pinv
        
    # Define state transition function for UKF
    def fx(self, x, dt):
        # x = [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, roll, pitch, yaw]
        # perf_counter position based on velocity and acceleration
        pos = x[:3] + x[3:6] * dt + 0.5 * x[6:] * dt**2
        
        # Update velocity based on acceleration
        vel = x[3:6] + x[6:] * dt
        
        # Orientation (roll, pitch, yaw) remains unchanged as we're not modeling its dynamics here
        orientation = x[6:]  
        
        # Return full state vector [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, roll, pitch, yaw]
        return np.hstack((pos, vel, orientation))



    # Define measurement function for UKF
    def hx(self, x):
        # Return the predicted accelerations and orientation (roll, pitch, yaw)
        acc_imu1_pred = x[3:6]  # From velocity state, representing accelerations
        acc_imu2_pred = x[3:6]  # Assuming similar accelerations for IMU 2
        orientation_pred = x[6:]  # roll, pitch, yaw from the state vector

        # Return combined predictions for accelerations and orientation
        return np.hstack((acc_imu1_pred, acc_imu2_pred, orientation_pred))
    
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
        self.ukf.Q = np.diag([position_variance[0], position_variance[1], position_variance[2],
                            velocity_variance[0], velocity_variance[1], velocity_variance[2],
                            orientation_variance[0], orientation_variance[1], orientation_variance[2]])*self.mulQ
        

        # Update measurement noise covariance R dynamically based on accelerometer and gyroscope variances
        self.ukf.R = np.diag([accel_variance[0], accel_variance[1], accel_variance[2],
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
        super().__init__('ukf_estimator')
        self.subscription_mpu = self.create_subscription(Mpu, 'mpu_data_2', self.listener_callback, 10)
        
        
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
        self.frec = 100
        self.kf = IMUFusionUKF(dt=1/self.frec )
        # Madgwick filter initialization
        self.madgwick_filter = Madgwick(frequency=self.frec ,gain=0.003)  # Adjust sample period as needed
        """
        If your IMU data is noisy, a lower beta value may help reduce jitter, though you will need to balance this with the slower data rate.
        """
        # Buffers to hold the last measurements from each MPU
        self.mpu1_data = None
        self.mpu2_data = None
        self.calibrated = False

        self.prev_time = perf_counter()
        self.prevOrientation  = np.array([0.0, 0.0, 0.0])
        #self.timerPrint = self.create_timer(1, self.printData)
        self.msg = COGframe()
        self.pos = np.array([0.0, 0.0, 0.0])
        self.orient = np.array([0.0, 0.0, 0.0])


    def listener_callback(self, msg):
        self.mpu1_data = msg
        self.mpu1_data.acy = self.mpu1_data.acy
        self.mpu1_data.acy2 = self.mpu1_data.acy2
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
        

        current_time = perf_counter()
        dt = current_time - self.prev_time
        if dt < 1e-6:
            dt = 1e-6  # Set a minimum time step threshold
         # Apply low-pass filter to the raw accelerometer data

        filtered_acx = self.low_pass_filter('acx', self.mpu1_data.acx)
        filtered_acy = self.low_pass_filter('acy', self.mpu1_data.acy)
        filtered_acz = self.low_pass_filter('acz', self.mpu1_data.acz)
        filtered_acx2 = self.low_pass_filter('acx', self.mpu1_data.acx2)
        filtered_acy2 = self.low_pass_filter('acy', self.mpu1_data.acy)
        filtered_acz2 = self.low_pass_filter('acz', self.mpu1_data.acz2)
        #self.kf.change_dt(dt)
        self.prev_time = current_time  # Update previous time
        # Add new measurement data to the buffers
        
        # Create the measurement vector with data from both IMUs
        
        # In process_fusion:
        
        alpha = 0.6  # Weight for IMU1, 1 - alpha for IMU2
        
        # Weighted average for gyroscope data in
        
        gyroscope_data_filtered = np.array([
            alpha * self.mpu1_data.gx + (1 - alpha) * self.mpu1_data.gx2,
            alpha * self.mpu1_data.gy + (1 - alpha) * self.mpu1_data.gy2,
            alpha * self.mpu1_data.gz + (1 - alpha) * self.mpu1_data.gz2
        ])*np.pi/180 # already in rad/s

        # Weighted average for accelerometer data
        accelerometer_data_filtered = np.array([
            alpha * filtered_acx + (1 - alpha) * filtered_acx2,
            alpha * filtered_acy + (1 - alpha) * filtered_acy2,
            alpha * filtered_acz + (1 - alpha) * filtered_acz2
        ])*9.81   # Convert to m/s²
        self.quaternion
        self.quaternion  = self.madgwick_filter.updateIMU(q=self.quaternion,gyr=gyroscope_data_filtered, acc=accelerometer_data_filtered)

       
        roll, pitch, yaw = self.quaternion_to_euler_angles(self.quaternion)  # in rads
        orientation = np.array([roll, pitch, yaw])
        
        # Compensate for gravity using the orientation from the Madgwick filter
        

        # ACY2 to much failed

        accel_imu1filt = np.array([filtered_acx, filtered_acy, filtered_acz]) 
        accel_imu2filt = np.array([filtered_acx2, filtered_acy, filtered_acz2]) 

       
        
        #orientation = self.kf.complementary_filter(accel_imu1, accel_imu2, gyro_imu1, gyro_imu2, dt, self.prevOrientation )
        #self.prevOrientation = orientation
        
        # Fused measurement vector for EKF (acceleration from both IMUs)
        z_imu1 = accel_imu1filt
        z_imu2 = accel_imu2filt
            # Fuse the accelerations (average)
        
        
        # Update the Kalman filter with
        # Prediction step
        self.kf.ukf.predict()

        # Update step with acceleration (linear velocity) and angular velocity
        measurements = np.hstack((z_imu1, z_imu2, [roll, pitch, yaw]))  # [vel_x, vel_y, vel_z, roll, pitch, yaw]
        # Add regularization to the measurement noise covariance
        self.kf.ukf.R += np.eye(self.kf.ukf.R.shape[0]) * 1e-6
        # Add regularization to the measurement noise covariance
        self.kf.ukf.Q += np.eye(self.kf.ukf.Q.shape[0]) * 1e-6

        self.kf.ukf.update(measurements)

        # Get filtered state (position, velocity, orientation)
        pos, vel, orient = self.kf.ukf.x[:3], self.kf.ukf.x[3:6], self.kf.ukf.x[6:9]

        
        self.add_measurement_to_buffers(self.mpu1_data,[roll, pitch, yaw],self.kf.ukf.x)
        self.update_measurement_noise()
        self.kf.update_noise_covariances(self.acc_variance, self.acc_variance2, self.gyro_variance, self.state_variance)
        
        # Publish the Kalman filter output
        msg = COGframe()
        msg.pos_x, msg.pos_y, msg.pos_z = float(pos[0]), float(pos[1]), float(pos[2])
        msg.roll, msg.pitch, msg.yaw = float(roll), float(pitch), float(yaw)
        self.publishKalmanFrame.publish(msg)
        self.pos = pos
        self.orient = orient
        

    def printData(self):

        #self.get_logger().info(f"Raw acc x y z: {float(self.mpu1_data.acx)}, {float(self.mpu1_data.acy)}, {float(self.mpu1_data.acz)}")
        #self.get_logger().info(f"Raw acc x y z 2: {float(self.mpu1_data.acx2)}, {float(self.mpu1_data.acy2)}, {float(self.mpu1_data.acz2)}")
        #self.get_logger().info(f"Raw gyro x y z: {float(self.mpu1_data.gx)}, {float(self.mpu1_data.gy)}, {float(self.mpu1_data.gz)}")
        #self.get_logger().info(f"Raw gyro x y z 2: {float(self.mpu1_data.gx2)}, {float(self.mpu1_data.gy2)}, {float(self.mpu1_data.gz2)}")
        self.get_logger().info(f"Pos X Y Z (meter) dt: {float(self.pos[0])}, {float(self.pos[1])}, {float(self.pos[2])},{1/self.kf.dt}")
        self.get_logger().info(f"Roll pitch yaw  : {float(self.orient[0])}, {float(self.orient[1])}, {float(self.orient[2])}")
        


def main(args=None):
    rclpy.init(args=args)
    ukf_estimator = CalCOGFrame()
    rclpy.spin(ukf_estimator)
    ukf_estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
