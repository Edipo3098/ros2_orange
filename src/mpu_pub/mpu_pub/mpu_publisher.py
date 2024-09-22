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
calibration_data1 = {
    "mpu1": {
        "accel": {"slope": [1.00287361, 0.99886526, 1.00341099], "offset": [0-.00174287, -0.00552432, 0.03603273]},
        "gyro": {"offset": [0.41042327880859375, 1.2082672119140625, 0.06053924560546875]}
    },
    "mpu2": {
        "accel": {"slope": [1.00038423, 1.00699974, 1.00256861 ], "offset": [0.02106848, 0.00323046,  0.05491282]},
        "gyro": {"offset": [0.9417724609375, 1.201019287109375, -1.0709762573242188]}
    }
}
# Constants for sensitivity values
ACCEL_SENSITIVITY = 2  # LSB/g for +/- 2g range
GYRO_SENSITIVITY = 250  # LSB/dps for +/- 250 dps range

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('mpu_publisher')
        self.publisher_ = self.create_publisher(Mpu, 'mpu_data_1', 10)
        

        # Publisher for Imu (standard message)
        self.imu_publisher_ = self.create_publisher(Imu, 'imu_data', 10)
        self.imu_publisher_second = self.create_publisher(Imu, 'imu_data_2', 10)
        self.i2c_bus = 1
        self.bus = smbus.SMBus(i2c_bus)
        # Then use self.bus.read_byte_data, etc. in the respective functions
        self.configure_sensor(mpu9250_address)
        self.configure_sensor(mpu9250_address_2)


        self.calibrationTime = time.time()
        self.current_time = time.time()
        

        self.check_full_scale(mpu9250_address)
        calibration_key = 'mpu1'
        
        self.calibrate_mpu(mpu9250_address,20000,calibration_key)

        calibration_key = 'mpu2'
        self.check_full_scale(mpu9250_address_2)
        self.calibrate_mpu(mpu9250_address_2,20000,calibration_key)
        

        self.Check_communication(mpu9250_address)
        self.Check_communication(mpu9250_address_2)
        timer_period = 1/100   # seconds 50Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.prev_accel_x, self.prev_accel_y, self.prev_accel_z = 0, 0, 0
        self.prev_gyro_x, self.prev_gyro_y, self.prev_gyro_z = 0, 0, 0

        self.prev_accel_x2, self.prev_accel_y2, self.prev_accel_z2 = 0, 0, 0
        self.prev_gyro_x2, self.prev_gyro_y2, self.prev_gyro_z2 = 0, 0, 0

    def check_full_scale(self, address):
        """Checks if the sensor is configured for full scale"""
        accel_config = self.bus.read_byte_data(address, 0x1C)
        gyro_config = self.bus.read_byte_data(address, 0x1B)

        accel_full_scale = (accel_config >> 3) & 0x03
        gyro_full_scale = (gyro_config >> 3) & 0x03

        if accel_full_scale != 0 or gyro_full_scale != 0:
            self.get_logger().info("Warning: Sensor is not configured for full scale (2g accel, 250 dps gyro).")

    def configure_sensor(self, address):
        """
        Configure the MPU9250 sensor for slow dynamics with low DLPF cutoff, 
        low sample rate, and normal power mode.
        """
        # Reset the device and wait for it to stabilize
        self.bus.write_byte_data(address, PWR_MGMT_1, 0x80)  # Reset the device
        time.sleep(0.1)

        # Exit sleep mode and set clock source to PLL with X-axis gyroscope reference
        self.bus.write_byte_data(address, PWR_MGMT_1, 0x00)
        time.sleep(0.1)

        # Set the DLPF to 20 Hz (DLPF_CFG = 0b100) in the CONFIG register
        self.bus.write_byte_data(address, CONFIG, 0x04)
        time.sleep(0.1)

        # Set the sample rate to 100 Hz (SMPLRT_DIV = 9)
        sample_rate_div = 9  # 100 Hz sample rate
        self.bus.write_byte_data(address, SMPLRT_DIV, sample_rate_div)
        time.sleep(0.1)

        # Set gyroscope sensitivity to ±250°/s
        gyro_config_sel = 0b00000  # Corresponds to ±250°/s
        self.bus.write_byte_data(address, GYRO_CONFIG, gyro_config_sel)
        time.sleep(0.1)

        # Set accelerometer sensitivity to ±2g
        accel_config_sel = 0b00000  # Corresponds to ±2g
        self.bus.write_byte_data(address, ACCEL_CONFIG, accel_config_sel)
        time.sleep(0.1)

        # Enable interrupts (optional depending on your use case)
        #self.bus.write_byte_data(address, INT_PIN_CFG, 0x22)  # Configure interrupt pin


    def Check_communication(self,address):
        try:
           
            who_am_i = self.bus.read_byte_data(address, MPU9250_WHO_AM_I)
            self.get_logger().info('I heard: "%s"' % hex(who_am_i))
        except Exception as e:
            self.get_logger().info(f'Error in communication: {e}')
        
    def mpu_wake(self, address):
        try:
            
            self.bus.write_byte_data(address, 0x6B, 0)
        except Exception as e:
            self.get_logger().info(f'Error in mpu_wake: {e}')
        
    def read_raw_data(self,address, reg,sensitivity):
        # Accelero and Gyro values are 16-bit
       
        high = self.bus.read_byte_data(address, reg)
        low = self.bus.read_byte_data(address, reg + 1)
        value = ((high << 8) | low)
        if value > 32768:
            value = value - 65536
        return value/sensitivity
    
    def calibrate_mpu(self,address, num_samples=10000,key='mpu1'):
        accel_data = []
        gyro_data = []
        accel_data_filtered = []
        
        self.calibrationTime = time.time()
        prev_accel_x, prev_accel_y, prev_accel_z = 0, 0, 0
        prev_gyro_x, prev_gyro_y, prev_gyro_z = 0, 0, 0
        self.get_logger().info(f"Calibrating MPU at address {hex(address)}...")
        finishCalibration = False
        firstCalibration = False
        #expected_gravity = np.array([1, 1 ,1])  # Assume gravity is in the negative z-axis is in g = 9.81 m/s²
        adjustment_factor = 0.01  # Small adjustment factor for adaptive calibration
        for _ in range(num_samples):
            try:
                accel_data_error = self.bus.read_i2c_block_data(address, 0x3B, 6)
                gyro_data_error = self.bus.read_i2c_block_data(address, 0x43, 6)
            except Exception as e:
                self.get_logger().info(f'Error in read_sensor_data: {e}')
                accel_data_error = [0, 0, 0, 0, 0, 0]
                gyro_data_error = [0, 0, 0, 0, 0, 0]

            accel_raw_data = accel_data_error 
            gyro_raw_data = gyro_data_error

            # Process accelerometer data
            accel_x = self.convert_data(accel_raw_data[0], accel_raw_data[1])
            accel_y = self.convert_data(accel_raw_data[2], accel_raw_data[3])
            accel_z = self.convert_data(accel_raw_data[4], accel_raw_data[5])

            # Process gyroscope data
            gyro_x = self.convert_data(gyro_raw_data[0], gyro_raw_data[1])
            gyro_y = self.convert_data(gyro_raw_data[2], gyro_raw_data[3])
            gyro_z = self.convert_data(gyro_raw_data[4], gyro_raw_data[5])

            accel_x = self.low_pass_filter(accel_x, prev_accel_x)
            accel_y = self.low_pass_filter(accel_y, prev_accel_y)
            accel_z = self.low_pass_filter(accel_z, prev_accel_z)

            gyro_x = self.low_pass_filter(gyro_x, prev_gyro_x)
            gyro_y = self.low_pass_filter(gyro_y, prev_gyro_y)
            gyro_z = self.low_pass_filter(gyro_z, prev_gyro_z)
            
            # Update previous values for next iteration
            prev_accel_x, prev_accel_y, prev_accel_z = accel_x, accel_y, accel_z
            prev_gyro_x, prev_gyro_y, prev_gyro_z = gyro_x, gyro_y, gyro_z

            #convert to acceleration in g and gyro dps
            accel_x = (accel_x/(2.0**15.0))*ACCEL_SENSITIVITY
            accel_y = (accel_y/(2.0**15.0))*ACCEL_SENSITIVITY
            accel_z = (accel_z/(2.0**15.0))*ACCEL_SENSITIVITY

            gyro_x = (gyro_x/(2.0**15.0))*GYRO_SENSITIVITY
            gyro_y = (gyro_y/(2.0**15.0))*GYRO_SENSITIVITY
            gyro_z = (gyro_z/(2.0**15.0))*GYRO_SENSITIVITY
            
            accel_data.append([accel_x, accel_y, accel_z])
            
            gyro_data.append([gyro_x, gyro_y, gyro_z])

            
    
        accel_data_array = np.array(accel_data)
        gyro_data_array = np.array(gyro_data)
        firstCalibration = True

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
        expected_gravity = np.array([-1,-1,-1])
        
        accel_slope = expected_gravity / np.mean(np.abs(accel_data_array), axis=0)
        accel_offset = accel_mean  # Use the mean as the offset
        accel_offset = accel_mean  # Use the mean as the offset

        # Gyroscope calibration (offset calculation)
        gyro_offset = np.mean(gyro_data_array, axis=0)
    
        mul = 1
        # Store calibration parameters
        if key == 'mpu2':
            mul = 0.5

        calibration_data[key]["accel"]["slope"] = accel_slope
        calibration_data[key]["accel"]["offset"] = (accel_offset*mul)
        calibration_data[key]["gyro"]["offset"] = gyro_offset

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
        """
        if self.check_calibration_convergence(sensor_data):
            return
        # Proceed with calibration adjustments if not converged
        accel_x, accel_y, accel_z = sensor_data[:3]

        # Expected values for accelerometer in a flat, stationary orientation are zero
        expected_accel_x, expected_accel_y, expected_accel_z = 0.0, 0.0, 0.0

        # Calculate the error between expected and actual values
        error_x = expected_accel_x - accel_x
        error_y = expected_accel_y - accel_y
        error_z = expected_accel_z - accel_z

        # Dynamically calculate adjustment factor based on error magnitude
        adjustment_factor_x = np.clip(abs(error_x), 0.001, 0.2)  # X-axis adjustment factor
        adjustment_factor_y = np.clip(abs(error_y), 0.001, 0.2)  # Y-axis adjustment factor
        adjustment_factor_z = np.clip(abs(error_z), 0.001, 0.2)  # Z-axis adjustment factor (Z may need larger adjustment)

        # Adjust the offsets based on error direction and magnitude
        if error_x > 0:
            # Positive error: measured value is too high, so decrease the offset
            calibration_data[calibration_key]["accel"]["offset"][0] -= adjustment_factor_x
        else:
            # Negative error: measured value is too low, so increase the offset
            calibration_data[calibration_key]["accel"]["offset"][0] += adjustment_factor_x

        if error_y > 0:
            # Positive error: measured value is too high, so decrease the offset
            calibration_data[calibration_key]["accel"]["offset"][1] -= adjustment_factor_y
        else:
            # Negative error: measured value is too low, so increase the offset
            calibration_data[calibration_key]["accel"]["offset"][1] += adjustment_factor_y

        if error_z > 0:
            # Positive error: measured value is too high, so decrease the offset
            calibration_data[calibration_key]["accel"]["offset"][2] -= adjustment_factor_z
        else:
            # Negative error: measured value is too low, so increase the offset
            calibration_data[calibration_key]["accel"]["offset"][2] += adjustment_factor_z

        # Log the calibration adjustments for debugging
        """
        self.get_logger().info(f"Adaptive calibration adjustment for {calibration_key}: "
                            f"Accel X offset: {calibration_data[calibration_key]['accel']['offset'][0]}, "
                            f"Accel Y offset: {calibration_data[calibration_key]['accel']['offset'][1]}, "
                            f"Accel Z offset: {calibration_data[calibration_key]['accel']['offset'][2]}")
        """

    def convert_data(self, high_byte, low_byte):
        """Converts high and low bytes into a signed integer"""
        value = (high_byte << 8) | low_byte
        if value > 32767:
            value -= 65536
        return value
    
    def low_pass_filter(self,current_value, previous_value, alpha=0.2):
        """Applies a low-pass filter to smooth raw sensor data."""
        return alpha * current_value + (1 - alpha) * previous_value
    def adaptive_calibration(self, sensor_data, calibration_key):
        """
        Perform adaptive calibration by continuously adjusting based on sensor data.
        - sensor_data: the current accelerometer data
        - calibration_key: 'mpu1' or 'mpu2'
        """
        accel_x, accel_y, accel_z = sensor_data[:3]

        # Expected values for accelerometer in a flat orientation (stationary)
        expected_accel_x, expected_accel_y, expected_accel_z = 0.0, 0.0, -1# delete gravity  # assuming gravity in z-axis only

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
        calibration_data[calibration_key]["accel"]["offset"] -= adjustment_factor * np.array([error_x, error_y, error_z])

        # Log adjustment (optional for debugging)
        self.get_logger().info(f"Adaptive calibration adjustment: {calibration_data[calibration_key]['accel']['offset']}")

    def read_sensor_data(self, address, calibration_key,prev):
        """Reads the raw sensor data from the MPU9250"""
        # Read accelerometer and gyroscope data
        try:
            accel_data_error = self.bus.read_i2c_block_data(address, 0x3B, 6)
            gyro_data_error = self.bus.read_i2c_block_data(address, 0x43, 6)
        except Exception as e:
            self.get_logger().info(f'Error in read_sensor_data: {e}')
            accel_data_error = [0, 0, 0, 0, 0, 0]
            gyro_data_error = [0, 0, 0, 0, 0, 0]

        accel_data = accel_data_error
        gyro_data = gyro_data_error

        # Process accelerometer data
        accel_x = self.convert_data(accel_data[0], accel_data[1])
        accel_y = self.convert_data(accel_data[2], accel_data[3])
        accel_z = self.convert_data(accel_data[4], accel_data[5])

        # Process gyroscope data
        gyro_x = self.convert_data(gyro_data[0], gyro_data[1])
        gyro_y = self.convert_data(gyro_data[2], gyro_data[3])
        gyro_z = self.convert_data(gyro_data[4], gyro_data[5])

        # Apply low-pass filter to raw values
        accel_x = self.low_pass_filter(accel_x, prev[0])
        accel_y = self.low_pass_filter(accel_y, prev[1])
        accel_z = self.low_pass_filter(accel_z, prev[2])

        gyro_x = self.low_pass_filter(gyro_x, prev[3])
        gyro_y = self.low_pass_filter(gyro_y, prev[4])
        gyro_z = self.low_pass_filter(gyro_z, prev[5])
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
        accel_x = (accel_x/(2.0**15.0))*ACCEL_SENSITIVITY
        accel_y = (accel_y/(2.0**15.0))*ACCEL_SENSITIVITY
        accel_z = (accel_z/(2.0**15.0))*ACCEL_SENSITIVITY

        gyro_x = (gyro_x/(2.0**15.0))*GYRO_SENSITIVITY
        gyro_y = (gyro_y/(2.0**15.0))*GYRO_SENSITIVITY
        gyro_z = (gyro_z/(2.0**15.0))*GYRO_SENSITIVITY
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
        msg = Mpu()
        try:
            self.current_time = time.time()
            if self.current_time - self.calibrationTime > 600:
                self.calibrate_mpu(mpu9250_address,20000,'mpu1')
                self.calibrate_mpu(mpu9250_address_2,20000,'mpu2')
            # Read accelerometer data
            key = 'mpu1'
            prev = [self.prev_accel_x, self.prev_accel_y, self.prev_accel_z, self.prev_gyro_x, self.prev_gyro_y, self.prev_gyro_z]
            accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.read_sensor_data(mpu9250_address, key, prev)
            # Apply adaptive calibration to adjust offsets in real-time
            self.adaptive_calibration([accel_x, accel_y, accel_z], key)

            self.prev_accel_x, self.prev_accel_y, self.prev_accel_z = accel_x, accel_y, accel_z
            self.prev_gyro_x, self.prev_gyro_y, self.prev_gyro_z = gyro_x, gyro_y, gyro_z
             # Update previous values for next loop iteration
            self.prev_accel_x, self.prev_accel_y, self.prev_accel_z = accel_x, accel_y, accel_z
            self.prev_gyro_x, self.prev_gyro_y, self.prev_gyro_z = gyro_x, gyro_y, gyro_z

            # Repeat the same process for the second MPU
            key = 'mpu2'
            prev = [self.prev_accel_x2, self.prev_accel_y2, self.prev_accel_z2, self.prev_gyro_x2, self.prev_gyro_y2, self.prev_gyro_z2]
            accel_x_2, accel_y_2, accel_z_2, gyro_x_2, gyro_y_2, gyro_z_2 = self.read_sensor_data(mpu9250_address_2, key, prev)

            # Apply adaptive calibration to MPU2
            self.adaptive_calibration([accel_x_2, accel_y_2, accel_z_2], key)

            # Update previous values for MPU2
            self.prev_accel_x2, self.prev_accel_y2, self.prev_accel_z2 = accel_x_2, accel_y_2, accel_z_2
            self.prev_gyro_x2, self.prev_gyro_y2, self.prev_gyro_z2 = gyro_x_2, gyro_y_2, gyro_z_2




            self.get_logger().info(f"Accel_x: {accel_x}, Accel_y: {accel_y}, Accel_z: {accel_z}")
            self.get_logger().info(f"Accel_x_2: {accel_x_2}, Accel_y_2: {accel_y_2}, Accel_z_2: {accel_z_2}")

            self.get_logger().info(f"Gyro_x: {gyro_x}, Gyro_y: {gyro_y}, Gyro_z: {gyro_z}")
            self.get_logger().info(f"Gyro_x_2: {gyro_x_2}, Gyro_y_2: {gyro_y_2}, Gyro_z_2: {gyro_z_2}")

            


            
            msg.message = "EL mensaje es"
            msg.acx = float(accel_x)
            msg.acy = float(accel_y)
            msg.acz = float(accel_z)
            msg.gx = float(gyro_x)
            msg.gy = float(gyro_y)
            msg.gz = float(gyro_z)

           
            
            msg.acx2 = float(accel_x_2)
            msg.acy2 = float(accel_y_2)
            msg.acz2 = float(accel_z_2)
            msg.gx2 = float(gyro_x_2)
            msg.gy2 = float(gyro_y_2)
            msg.gz2 = float(gyro_z_2)
            # Convert to Imu message and publish
            #imu_msg_1 = self.convert_mpu_to_imu(msg)  # First sensor
            #imu_msg_2 = self.convert_mpu_to_imu(msg2)  # Second sensor

            #self.imu_publisher_.publish(imu_msg_1)
            #self.imu_publisher_second.publish(imu_msg_2)
            self.publisher_.publish(msg)
            
            #self.get_logger().info('is publishing')

        except KeyboardInterrupt:
            self.get_logger().info('Exiting the system key')
        #finally:
            #self.get_logger().info('Exiting the system')



def main(args=None):
    rclpy.init(args=args)

    mpu_publisher = MinimalPublisher()

    rclpy.spin(mpu_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mpu_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
