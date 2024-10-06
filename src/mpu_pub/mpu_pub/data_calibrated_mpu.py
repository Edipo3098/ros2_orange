# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
import smbus
import time
from std_msgs.msg import String
from robot_interfaces.msg import Mpu
import numpy as np
from sensor_msgs.msg import Imu
from time import perf_counter
from rclpy.executors import MultiThreadedExecutor
import numpy as np
from collections import deque

mpu9250_address = 0x68  # MPU9250 default I2C address
mpu9250_address_2 = 0x69  # MPU9250 I2C address AD0 high
PWR_MGMT_1 = 0x6B
# Create an smbus object
i2c_bus = 1  # Assuming you want to use /dev/i2c-1


# MPU9250 register addresses
MPU9250_WHO_AM_I = 0x75
WR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
INT_PIN_CFG  = 0x37
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
TEMP_OUT_H   = 0x41
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
# Calibration data for two MPU9250s
calibration_data = {
    "mpu1": {
        "accel": {"slope": [-1.00287361, -0.99886526, -1.00341099], "offset": [0.00174287, 0.00552432, -0.03603273]},
        "gyro": {"offset": [0.41042327880859375, 1.2082672119140625, 0.06053924560546875]}
    },
    "mpu2": {
        "accel": {"slope": [-1.00038423, -1.00699974, -1.00256861 ], "offset": [-0.02106848, -0.00323046,  -0.05491282]},
        "gyro": {"offset": [0.9417724609375, 1.201019287109375, -1.0709762573242188]}
    }
}
MPU_CONFIG = {
    "ACCEL_SENSITIVITY": 2,  # LSB/g for +/- 2g
    "GYRO_SENSITIVITY": 250  # LSB/dps for +/- 250 dps
}

# Constants for sensitivity values
ACCEL_SENSITIVITY_NORM = 2 
GYRO_SENSITIVITY_NORM = 250

ACCEL_SENSITIVITY_RAW = 16384
GYRO_SENSITIVITY_RAW = 131 
ACCEL_SENSITIVITY = ACCEL_SENSITIVITY_RAW  # LSB/g for +/- 2g range
GYRO_SENSITIVITY = GYRO_SENSITIVITY_RAW 



class KalmanFilter:
    def __init__(self, Q=0.0001, R=0.01):
        # Process noise covariance (Q) and measurement noise covariance (R)
        self.Q = Q
        self.R = R
        self.x = 0.0  # State estimate
        self.P = 1.0  # Estimate covariance

    def update(self, measurement):
        # Prediction step
        self.P += self.Q

        # Measurement update step
        K = self.P / (self.P + self.R)  # Kalman Gain
        self.x += K * (measurement - self.x)  # Update estimate with measurement
        self.P *= (1 - K)  # Update estimate covariance

        return self.x

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('data_calibrated_mpu')
        self.subscriber = self.create_subscription(Mpu, 'mpu_data_1',self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Mpu, 'mpu_data_2', 10) # In gravity and degree*second
        # Kalman Filters for each axis of accelerometer and gyroscope
        self.kf_accel_x = KalmanFilter()
        self.kf_accel_y = KalmanFilter()
        self.kf_accel_z = KalmanFilter()
        self.kf_gyro_x = KalmanFilter()
        self.kf_gyro_y = KalmanFilter()
        self.kf_gyro_z = KalmanFilter()

        # Kalman Filters for second MPU
        self.kf_accel_x2 = KalmanFilter()
        self.kf_accel_y2 = KalmanFilter()
        self.kf_accel_z2 = KalmanFilter()
        self.kf_gyro_x2 = KalmanFilter()
        self.kf_gyro_y2 = KalmanFilter()
        self.kf_gyro_z2 = KalmanFilter()

        self.current_time = perf_counter()
        self.prevTime = perf_counter()
        self.dt = self.current_time - self.prevTime 

        self.current_time2 = perf_counter()
        self.prevTime2 = perf_counter()
        self.dt2 = self.current_time2 - self.prevTime2 

        self.msg = Mpu()
        self.msg2 = Mpu()
  

        


        self.calibrationTime = time.time()
        self.current_time = time.time()
              
        timer_period = 1/2000   # seconds 50Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.sendData = self.create_timer(timer_period, self.send_data)
        self.timer2 = self.create_timer(1, self.timer_callback2)
        self.i = 0
        self.prev_accel_x, self.prev_accel_y, self.prev_accel_z = 0, 0, 0
        self.prev_gyro_x, self.prev_gyro_y, self.prev_gyro_z = 0, 0, 0

        self.prev_accel_x2, self.prev_accel_y2, self.prev_accel_z2 = 0, 0, 0
        self.prev_gyro_x2, self.prev_gyro_y2, self.prev_gyro_z2 = 0, 0, 0
        self.window_size = 1000
        self.acc_buffers = {
            'acx': deque(maxlen=self.window_size),
            'acy': deque(maxlen=self.window_size),
            'acz': deque(maxlen=self.window_size),
            'acx2': deque(maxlen=self.window_size),
            'acy2': deque(maxlen=self.window_size),
            'acz2': deque(maxlen=self.window_size),
        }
        self.gyro_buffers = {
            'roll': deque(maxlen=self.window_size),
            'pitch': deque(maxlen=self.window_size),
            'yaw': deque(maxlen=self.window_size),
            'roll2': deque(maxlen=self.window_size),
            'pitch2': deque(maxlen=self.window_size),
            'yaw2': deque(maxlen=self.window_size),
        }
        
        self.dataCalibrated = False
        self.get_logger().info(f"Calibrating MPU at address ...")
        self.current_accel,self.current_gyro = np.array([0,0,0]),np.array([0,0,0])
        self.current_accel2,self.current_gyro2 =np.array([0,0,0]),np.array([0,0,0]) 
   

    def listener_callback(self, msg):
        acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, acc_x2, acc_y2, acc_z2, gyro_x2, gyro_y2, gyro_z2 = self.convertDate(msg)
        if not self.dataCalibrated:
            
            self.acc_buffers['acx'].append(acc_x)
            self.acc_buffers['acy'].append(acc_y)
            self.acc_buffers['acz'].append(acc_z)
            self.acc_buffers['acx2'].append(acc_x2)
            self.acc_buffers['acy2'].append(acc_y2)
            self.acc_buffers['acz2'].append(acc_z2)
            self.gyro_buffers['roll'].append(gyro_x)
            self.gyro_buffers['pitch'].append(gyro_y)
            self.gyro_buffers['yaw'].append(gyro_z)
            self.gyro_buffers['roll2'].append(gyro_x2)
            self.gyro_buffers['pitch2'].append(gyro_y2)
            self.gyro_buffers['yaw2'].append(gyro_z2)
            
            self.calibrate_mpu()
            return
        else:
            self.current_accel = np.array([acc_x, acc_y, acc_z])
            self.current_gyro = np.array([gyro_x, gyro_y, gyro_z])
            self.current_accel2 = np.array([acc_x2, acc_y2, acc_z2])
            self.current_gyro2 = np.array([gyro_x2, gyro_y2, gyro_z2])
    def convertDate(self,msg):
        accel_x_filtered = self.kf_accel_x.update(msg.acx)
        accel_y_filtered = self.kf_accel_y.update(msg.acy)
        accel_z_filtered = self.kf_accel_z.update(msg.acz)

        gyro_x_filtered = self.kf_gyro_x.update(msg.gx)
        gyro_y_filtered = self.kf_gyro_y.update(msg.gy)
        gyro_z_filtered = self.kf_gyro_z.update(msg.gz)

        accel_x_filtered2 = self.kf_accel_x.update(msg.acx2)
        accel_y_filtered2 = self.kf_accel_y.update(msg.acy2)
        accel_z_filtered2 = self.kf_accel_z.update(msg.acz2)

        gyro_x_filtered2 = self.kf_gyro_x.update(msg.gx2)
        gyro_y_filtered2 = self.kf_gyro_y.update(msg.gy2)
        gyro_z_filtered2 = self.kf_gyro_z.update(msg.gz2)
        
        

        accel_x = (accel_x_filtered/ACCEL_SENSITIVITY)
        accel_y = (accel_y_filtered/ACCEL_SENSITIVITY)
        accel_z = (accel_z_filtered/ACCEL_SENSITIVITY)

        gyro_x = (gyro_x_filtered/GYRO_SENSITIVITY)
        gyro_y = (gyro_y_filtered/GYRO_SENSITIVITY)
        gyro_z = (gyro_z_filtered/GYRO_SENSITIVITY)

        accel_x2 = (accel_x_filtered2/ACCEL_SENSITIVITY)
        accel_y2 = (accel_y_filtered2/ACCEL_SENSITIVITY)
        accel_z2 = (accel_z_filtered2/ACCEL_SENSITIVITY)

        gyro_x2 = (gyro_x_filtered2/GYRO_SENSITIVITY)
        gyro_y2 = (gyro_y_filtered2/GYRO_SENSITIVITY)
        gyro_z2 = (gyro_z_filtered2/GYRO_SENSITIVITY)

        return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, accel_x2, accel_y2, accel_z2, gyro_x2, gyro_y2, gyro_z2


    def calibrate_mpu(self, num_samples=10000):
        accel_data = []
        gyro_data = []
        accel_data_filtered = []
        
        self.calibrationTime = time.time()
        prev_accel_x, prev_accel_y, prev_accel_z = 0, 0, 0
        prev_gyro_x, prev_gyro_y, prev_gyro_z = 0, 0, 0

       
        
        finishCalibration = False
        firstCalibration = False
        #expected_gravity = np.array([1, 1 ,1])  # Assume gravity is in the negative z-axis is in g = 9.81 m/s²
        adjustment_factor = 0.01  # Small adjustment factor for adaptive calibration
        if len(self.acc_buffers['acx'])>= self.window_size :
        
            
            # Assuming your deques for accelerometer and gyroscope data
            accel_data_x = np.array(self.acc_buffers['acx'])
            accel_data_y = np.array(self.acc_buffers['acy'])
            accel_data_z = np.array(self.acc_buffers['acz'])

            # Combine accelerometer data into one array for processing
            accel_data_array = np.stack((accel_data_x, accel_data_y, accel_data_z), axis=1)

            accel_data_x2 = np.array(self.acc_buffers['acx2'])
            accel_data_y2 = np.array(self.acc_buffers['acy2'])
            accel_data_z2 = np.array(self.acc_buffers['acz2'])

            # Combine accelerometer data into one array for processing
            accel_data_array2 = np.stack((accel_data_x2, accel_data_y2, accel_data_z2), axis=1)

            # Assuming your deques for accelerometer and gyroscope data
            gyro_data_roll = np.array(self.gyro_buffers['roll'])
            gyro_data_pitch = np.array(self.gyro_buffers['pitch'])
            gyro_data_yaw = np.array(self.gyro_buffers['yaw2'])

            # Combine accelerometer data into one array for processing
            gyro_data_array = np.stack((gyro_data_roll, gyro_data_pitch, gyro_data_yaw), axis=1)

            gyro_data_roll2 = np.array(self.gyro_buffers['roll2'])
            gyro_data_pitch2 = np.array(self.gyro_buffers['pitch2'])
            gyro_data_yaw2 = np.array(self.gyro_buffers['yaw2'])

            # Combine accelerometer data into one array for processing
            gyro_data_array2 = np.stack((gyro_data_roll2, gyro_data_pitch2, gyro_data_yaw2), axis=1)

            self.calculateCalibrations(accel_data_array,gyro_data_array,'mpu1')
            self.calculateCalibrations(accel_data_array2,gyro_data_array2,'mpu2')

            self.dataCalibrated = True
    def calculateCalibrations(self,accel_data,gyro_data,key):
    
        accel_data_array = np.array(accel_data)
        gyro_data_array = np.array(gyro_data)
        

        # Accelerometer calibration (slope and offset calculation using linear regression)
        accel_mean = np.mean(accel_data_array, axis=0)
        accel_std = np.std (accel_data_array, axis=0)
        accel_min = np.min(accel_data_array, axis=0)
        accel_max = np.max(accel_data_array, axis=0)

        accel_slope = np.ones(3)  # Slope should generally be near 1 for accelerometer
        # Compute slope by comparing accelerometer values to expected values (e.g., 1g)
        # Expected value is 9.81 m/s² when axis is aligned with gravity
        # Dynamically adjust only the Z-axis expected gravity based on real-time measurements
        #current_accel_z = np.mean(np.abs(accel_data[:, 2]))
        
        #expected_gravity[2] += adjustment_factor * (current_accel_z - np.abs(expected_gravity[2]))
        expected_gravity = np.array([0,0,-1])
        
        accel_slope = expected_gravity / np.mean(np.abs(accel_data_array), axis=0)
        accel_offset = accel_mean  # Use the mean as the offset
        accel_offset = accel_mean  # Use the mean as the offset

        # Gyroscope calibration (offset calculation)
        gyro_offset = np.mean(gyro_data_array, axis=0)
    
        mul = 1
       
        # Store calibration parameters
        if key == 'mpu2':
            mul = 1

        calibration_data[key]["accel"]["slope"] = accel_slope
        calibration_data[key]["accel"]["offset"] = (accel_offset*mul)
        calibration_data[key]["gyro"]["offset"] = gyro_offset
        key == 'mpu2'
        # Log the standard deviation for diagnostic purposes
        # You can also compute standard deviation and discard high variance data
        accel_std = np.std(accel_data, axis=0)
        gyro_std = np.std(gyro_data, axis=0)
                
        self.get_logger().info(f"Accel standard deviation: {accel_std}, Gyro standard deviation: {gyro_std}")

        self.get_logger().info(f"Calibration completed for {key}. Accel offsets: {accel_offset}, Gyro offsets: {gyro_offset}")
    def check_calibration_convergence(self, sensor_data, threshold=0.001):
        """
        Check if the sensor data has converged to within a small threshold for all axes.
        - sensor_data: the current accelerometer data (X, Y, Z)
        - threshold: the acceptable error margin to consider convergence (default is 0.05)
        """
        accel_x, accel_y, accel_z = sensor_data[:3]

        # Expected values for accelerometer in a flat orientation (stationary) are zero
        expected_accel_x, expected_accel_y, expected_accel_z = 0.0, 0.0, 0.0

        # Calculate the absolute errors
        error_x = abs(expected_accel_x - accel_x)
        error_y = abs(expected_accel_y - accel_y)
        error_z = abs(expected_accel_z - accel_z)

        # Check if all errors are within the acceptable threshold
        if error_x < threshold and error_y < threshold and error_z < threshold:
            return True  # Calibration has converged
        return False  # Still need to adjust calibration

    def adaptive_calibration2(self, sensor_data, calibration_key):
        """
        Perform adaptive calibration by continuously adjusting based on sensor data.
        The expected value for each axis is assumed to be zero.
        - sensor_data: the current accelerometer data (X, Y, Z)
        - calibration_key: 'mpu1' or 'mpu2'
        
        if self.check_calibration_convergence(sensor_data):
            return

        """
        # Proceed with calibration adjustments if not converged
        accel_x, accel_y, accel_z = sensor_data[:3]

        # Expected values for accelerometer in a flat, stationary orientation are zero
        expected_accel_x, expected_accel_y, expected_accel_z = 0.0, 0.0, 0.0

        # Calculate the error between expected and actual values
        error_x = expected_accel_x - accel_x
        error_y = expected_accel_y - accel_y
        error_z = expected_accel_z - accel_z

        # Dynamically calculate adjustment factor based on error magnitude
        adjustment_factor_x = np.clip(abs(error_x), 0.001, 0.02)  # X-axis adjustment factor
        adjustment_factor_y = np.clip(abs(error_y), 0.001, 0.02)  # Y-axis adjustment factor
        adjustment_factor_z = np.clip(abs(error_z), 0.001, 0.02)  # Z-axis adjustment factor (Z may need larger adjustment)

        # Adjust the offsets based on error direction and magnitude
        
        if error_x >=  0.05:
            # Positive error: measured value is too high, so decrease the offset
            calibration_data[calibration_key]["accel"]["offset"][0] += adjustment_factor_x
        else:
            if error_x <= -0.05:
                # Negative error: measured value is too low, so increase the offset
                calibration_data[calibration_key]["accel"]["offset"][0] -= adjustment_factor_x

        if error_y >=  0.05:
            # Positive error: measured value is too high, so decrease the offset
            calibration_data[calibration_key]["accel"]["offset"][1] += adjustment_factor_y
        else:
            if error_y <= -0.05:
                # Negative error: measured value is too low, so increase the offset
                calibration_data[calibration_key]["accel"]["offset"][1] -= adjustment_factor_y
        
        if error_z >=  0.05:
            # Positive error: measured value is too high, so decrease the offset
            calibration_data[calibration_key]["accel"]["offset"][2] += adjustment_factor_z
        else:
            if error_z <= -0.05:
                # Negative error: measured value is too low, so increase the offset
                calibration_data[calibration_key]["accel"]["offset"][2] -= adjustment_factor_z

        #self.get_logger().info(f"Adaptive calibration adjustment 2: {error_z}")
        # Log the calibration adjustments for debugging
        """
        self.get_logger().info(f"Adaptive calibration adjustment for {calibration_key}: "
                            f"Accel X offset: {calibration_data[calibration_key]['accel']['offset'][0]}, "
                            f"Accel Y offset: {calibration_data[calibration_key]['accel']['offset'][1]}, "
                            f"Accel Z offset: {calibration_data[calibration_key]['accel']['offset'][2]}")
        """

   
    
    def low_pass_filter(self, current_value, previous_value, alpha=None):
        if alpha is None:
            alpha = self.dynamic_alpha_calculation(current_value, previous_value)
        return alpha * current_value + (1 - alpha) * previous_value
    
    def dynamic_alpha_calculation(self, current_value, previous_value):
        # Example: Calculate alpha based on the difference between values
        delta = abs(current_value - previous_value)
        return min(0.9, max(0.1, delta / 100))  # Adjust range as needed
    def adaptive_calibration(self, sensor_data, calibration_key):
        """
        Perform adaptive calibration by continuously adjusting based on sensor data.
        - sensor_data: the current accelerometer data
        - calibration_key: 'mpu1' or 'mpu2'
        """
        accel_x, accel_y, accel_z = sensor_data[:3]

        # Expected values for accelerometer in a flat orientation (stationary)
        expected_accel_x, expected_accel_y, expected_accel_z = 0.0, 0.0, 0.0# delete gravity  # assuming gravity in z-axis only

        # Calculate the error in measurement
        error_x = expected_accel_x - accel_x
        error_y = expected_accel_y - accel_y
        error_z = expected_accel_z - accel_z

        # Dynamically adjust the calibration offset (small adjustment factor to avoid over-adjusting)
        adjustment_factor = np.array([0.01 , 0.01 ,0.01])  # Small adjustment factor for adaptive calibration
        
        if (error_x<0.1):
            adjustment_factor[0]  = 0.0000
        if (error_y<0.1):
            adjustment_factor[1]  = 0.0000
        if (error_z<0.1):
            adjustment_factor[2]  = 0.0000
        calibration_data[calibration_key]["accel"]["offset"][0]+= adjustment_factor[0] * (error_x)
        #calibration_data[calibration_key]["accel"]["offset"][1]+= adjustment_factor[1] * (error_y)
        
        
        # Define a threshold for error below which no adjustment should be made
        threshold = 0.05
        """
        # Only adjust calibration offset if the error exceeds the threshold
        if abs(error_x) > threshold:
            calibration_data[calibration_key]["accel"]["offset"][0] += 0.01 * error_x
        if abs(error_y) > threshold:
            calibration_data[calibration_key]["accel"]["offset"][1] += 0.01 * error_y
        if abs(error_z) > threshold:
            calibration_data[calibration_key]["accel"]["offset"][2] += 0.01 * error_z
        # Log adjustment (optional for debugging)
        """
        #self.get_logger().info(f"Adaptive calibration adjustment 1: {calibration_data[calibration_key]['accel']['offset']}")

    def read_sensor_data(self, accel, gyro, calibration_key,prev):
        """Reads the raw sensor data from the MPU9250"""
        accel_x =accel[0]
        accel_y = accel[1]
        accel_z = accel[2]
        gyro_x = gyro[0]
        gyro_y = gyro[1]
        gyro_z = gyro[2]
        # Apply Kalman filter to smooth data

        accel_x = self.low_pass_filter(accel_x, prev[0])
        accel_y = self.low_pass_filter(accel_y, prev[1])
        accel_z = self.low_pass_filter(accel_z, prev[2])

        gyro_x = self.low_pass_filter(gyro_x, prev[3])
        gyro_y = self.low_pass_filter(gyro_y, prev[4])
        gyro_z = self.low_pass_filter(gyro_z, prev[5])

        accel_y_filtered = accel_y
        accel_x_filtered = accel_x
        accel_z_filtered = accel_z

        gyro_x_filtered =gyro_x
        gyro_y_filtered = gyro_y
        gyro_z_filtered = gyro_z
        """accel_x_filtered = self.kf_accel_x.update(accel_x)
        accel_y_filtered = self.kf_accel_y.update(accel_y)
        accel_z_filtered = self.kf_accel_z.update(accel_z)

        gyro_x_filtered = self.kf_gyro_x.update(gyro_x)
        gyro_y_filtered = self.kf_gyro_y.update(gyro_y)
        gyro_z_filtered = self.kf_gyro_z.update(gyro_z)"""
        """
        # Apply low-pass filter to raw values
        
        """
        # Apply low-pass filter to raw values
        #accel_x = self.low_pass_filter(accel_x, prev[0])
        #accel_y = self.low_pass_filter(accel_y, prev[1])
        #accel_z = self.low_pass_filter(accel_z, prev[2])

        #gyro_x = self.low_pass_filter(gyro_x, prev[3])
        #gyro_y = self.low_pass_filter(gyro_y, prev[4])
        #gyro_z = self.low_pass_filter(gyro_z, prev[5])

        
        # Convert to correct units
            # Convert to correct units
        #convert to acceleration in g and gyro dps
        """
            accel_x = (accel_x_filtered/(2.0**15.0))*ACCEL_SENSITIVITY
            accel_y = (accel_y_filtered/(2.0**15.0))*ACCEL_SENSITIVITY
            accel_z = (accel_z_filtered/(2.0**15.0))*ACCEL_SENSITIVITY

            gyro_x = (gyro_x_filtered/(2.0**15.0))*GYRO_SENSITIVITY
            gyro_y = (gyro_z_filtered/(2.0**15.0))*GYRO_SENSITIVITY
            gyro_z = (gyro_z_filtered/(2.0**15.0))*GYRO_SENSITIVITY

            """

        accel_x = (accel_x_filtered/ACCEL_SENSITIVITY)
        accel_y = (accel_y_filtered/ACCEL_SENSITIVITY)
        accel_z = (accel_z_filtered/ACCEL_SENSITIVITY)

        gyro_x = (gyro_x_filtered/GYRO_SENSITIVITY)
        gyro_y = (gyro_y_filtered/GYRO_SENSITIVITY)
        gyro_z = (gyro_z_filtered/GYRO_SENSITIVITY)
        # Apply calibration
        """
        accel_x = (accel_x - calibration_data[calibration_key]["accel"]["offset"][0])    + calibration_data[calibration_key]["accel"]["slope"][0]
        accel_y = ( accel_y - calibration_data[calibration_key]["accel"]["offset"][1])    +calibration_data[calibration_key]["accel"]["slope"][1]
        accel_z = ( accel_z - calibration_data[calibration_key]["accel"]["offset"][2])    +calibration_data[calibration_key]["accel"]["slope"][2]
        
        """
        # Apply calibration (common form: subtract offset, then apply slope)
        accel_x = (accel_x - calibration_data[calibration_key]["accel"]["offset"][0]) * calibration_data[calibration_key]["accel"]["slope"][0]
        accel_y = (accel_y - calibration_data[calibration_key]["accel"]["offset"][1]) * calibration_data[calibration_key]["accel"]["slope"][1]
        accel_z = (accel_z - calibration_data[calibration_key]["accel"]["offset"][2]) * calibration_data[calibration_key]["accel"]["slope"][2]

        gyro_x -= calibration_data[calibration_key]["gyro"]["offset"][0]
        gyro_y -= calibration_data[calibration_key]["gyro"]["offset"][1]
        gyro_z -= calibration_data[calibration_key]["gyro"]["offset"][2]


        
        

        return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

   
    
    def timer_callback(self):
        self.current_time = perf_counter()
        
        self.dt = self.current_time - self.prevTime 

        self.prevTime = perf_counter()
        if not self.dataCalibrated:
            
            return
        else:
        
            # Read accelerometer data
            key = 'mpu1'
            prev = [self.prev_accel_x, self.prev_accel_y, self.prev_accel_z, self.prev_gyro_x, self.prev_gyro_y, self.prev_gyro_z]
            accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.read_sensor_data(self.current_accel,self.current_gyro, key, prev)
            
            # Apply adaptive calibration to adjust offsets in real-time
            self.adaptive_calibration([accel_x, accel_y, accel_z], key)
            self.adaptive_calibration2([accel_x, accel_y, accel_z], key) # Second calibration method to Z axis

            self.prev_accel_x, self.prev_accel_y, self.prev_accel_z = accel_x, accel_y, accel_z
            self.prev_gyro_x, self.prev_gyro_y, self.prev_gyro_z = gyro_x, gyro_y, gyro_z
             # Update previous values for next loop iteration
            self.prev_accel_x, self.prev_accel_y, self.prev_accel_z = accel_x, accel_y, accel_z
            self.prev_gyro_x, self.prev_gyro_y, self.prev_gyro_z = gyro_x, gyro_y, gyro_z

            # Repeat the same process for the second MPU
            key = 'mpu2'
            prev = [self.prev_accel_x2, self.prev_accel_y2, self.prev_accel_z2, self.prev_gyro_x2, self.prev_gyro_y2, self.prev_gyro_z2]
            accel_x_2, accel_y_2, accel_z_2, gyro_x_2, gyro_y_2, gyro_z_2 = self.read_sensor_data(self.current_accel2,self.current_gyro2, key, prev)

            # Apply adaptive calibration to MPU2
            self.adaptive_calibration([accel_x_2, accel_y_2, accel_z_2], key)
            self.adaptive_calibration2([accel_x_2, accel_y_2, accel_z_2], key) # Second calibration method to Z axis

            # Update previous values for MPU2
            self.prev_accel_x2, self.prev_accel_y2, self.prev_accel_z2 = accel_x_2, accel_y_2, accel_z_2
            self.prev_gyro_x2, self.prev_gyro_y2, self.prev_gyro_z2 = gyro_x_2, gyro_y_2, gyro_z_2


            #self.get_logger().info('Exiting the system key')
        #finally:
            #self.get_logger().info('Exiting the system')
    def timer_callback2(self):
        return
        self.get_logger().info(f"Accel_x: {self.prev_accel_x}, Accel_y: {self.prev_accel_y}, Accel_z: {self.prev_accel_z}")
        self.get_logger().info(f"Accel_x_2: {self.prev_accel_x2}, Accel_y_2: {self.prev_accel_y2}, Accel_z_2: {self.prev_accel_z2}")
        self.get_logger().info(f"delta time process  {self.dt }")
        #self.get_logger().info(f"delta time sen data {self.dt2 }")
    def send_data(self):
        self.current_time2 = perf_counter()
        
        self.dt2 = self.current_time2 - self.prevTime2 

        self.prevTime2= perf_counter()
        
        self.msg2.message = "EL mensaje es"
        self.msg2.acx = float(self.prev_accel_x)
        self.msg2.acy = float(self.prev_accel_y)
        self.msg2.acz = float(self.prev_accel_z)
        self.msg2.gx = float(self.prev_gyro_x)
        self.msg2.gy = float(self.prev_gyro_y)
        self.msg2.gz = float(self.prev_gyro_z)

        
        
        self.msg2.acx2 = float(self.prev_accel_x2)
        self.msg2.acy2 = float(self.prev_accel_y2)
        self.msg2.acz2 = float(self.prev_accel_z2)
        self.msg2.gx2 = float(self.prev_gyro_x2)
        self.msg2.gy2 = float(self.prev_gyro_y2)
        self.msg2.gz2 = float(self.prev_gyro_z2)
        # Convert to Imu message and publish
        #imu_msg_1 = self.convert_mpu_to_imu(msg)  # First sensor
        #imu_msg_2 = self.convert_mpu_to_imu(msg2)  # Second sensor

        #self.imu_publisher_.publish(imu_msg_1)
        #self.imu_publisher_second.publish(imu_msg_2)
        self.publisher_.publish(self.msg2)

def main(args=None):
    
    
    rclpy.init(args=args)
    mpu_publisher = MinimalPublisher()
    executor = MultiThreadedExecutor()
    executor.add_node(mpu_publisher)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        mpu_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
