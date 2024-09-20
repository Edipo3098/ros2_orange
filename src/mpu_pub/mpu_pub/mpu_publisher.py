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
calibration_data1 = {
    "mpu1": {
        "accel": {"slope": [-1.00287361, -0.99886526, -1.00341099], "offset": [0.00174287, 0.00552432, -0.03603273]},
        "gyro": {"offset": [0.41042327880859375, 1.2082672119140625, 0.06053924560546875]}
    },
    "mpu2": {
        "accel": {"slope": [-1.00038423, -1.00699974, -1.00256861 ], "offset": [-0.02106848, -0.00323046,  -0.05491282]},
        "gyro": {"offset": [0.9417724609375, 1.201019287109375, -1.0709762573242188]}
    }
}
calibration_data = {
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
        
        #self.calibrate_mpu(mpu9250_address,20000,calibration_key)

        calibration_key = 'mpu2'
        self.check_full_scale(mpu9250_address_2)
        #self.calibrate_mpu(mpu9250_address_2,20000,calibration_key)
        

        self.Check_communication(mpu9250_address)
        self.Check_communication(mpu9250_address_2)
        timer_period = 1/50   # seconds 50Hz
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
        self.bus.write_byte_data(address,PWR_MGMT_1,0x80)
        time.sleep(0.1)
        self.bus.write_byte_data(address,PWR_MGMT_1,0x00)
        time.sleep(0.1)
        # power management and crystal settings
        self.bus.write_byte_data(address, PWR_MGMT_1, 0x01)
        time.sleep(0.1)
        # alter sample rate (stability)
        samp_rate_div = 0 # sample rate = 8 kHz/(1+samp_rate_div)
        self.bus.write_byte_data(address, SMPLRT_DIV, samp_rate_div)
        time.sleep(0.1)
        #Write to Configuration register
        self.bus.write_byte_data(address, CONFIG, 0)
        time.sleep(0.1)
        #Write to Gyro configuration register
        gyro_config_sel = [0b00000,0b01000,0b10000,0b11000] # byte registers
        gyro_config_vals = [250.0,500.0,1000.0,2000.0] # degrees/sec
        gyro_indx = 0
        self.bus.write_byte_data(address, GYRO_CONFIG, int(gyro_config_sel[gyro_indx]))
        time.sleep(0.1)
        #Write to Accel configuration register
        accel_config_sel = [0b00000,0b01000,0b10000,0b11000] # byte registers
        accel_config_vals = [2.0,4.0,8.0,16.0] # g (g = 9.81 m/s^2)
        accel_indx = 0
        self.bus.write_byte_data(address, ACCEL_CONFIG, int(accel_config_sel[accel_indx]))
        time.sleep(0.1)
        # interrupt register (related to overflow of data [FIFO])
        self.bus.write_byte_data(address,INT_PIN_CFG,0x22)

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
        self.calibrationTime = time.time()
        prev_accel_x, prev_accel_y, prev_accel_z = 0, 0, 0
        prev_gyro_x, prev_gyro_y, prev_gyro_z = 0, 0, 0
        self.get_logger().info(f"Calibrating MPU at address {hex(address)}...")

        for _ in range(num_samples):
            accel_raw_data = self.bus.read_i2c_block_data(address, 0x3B, 6)
            gyro_raw_data = self.bus.read_i2c_block_data(address, 0x43, 6)

            # Process accelerometer data
            accel_x = self.convert_data(accel_raw_data[0], accel_raw_data[1])
            accel_y = self.convert_data(accel_raw_data[2], accel_raw_data[3])
            accel_z = self.convert_data(accel_raw_data[4], accel_raw_data[5])

            # Process gyroscope data
            gyro_x = self.convert_data(gyro_raw_data[0], gyro_raw_data[1])
            gyro_y = self.convert_data(gyro_raw_data[2], gyro_raw_data[3])
            gyro_z = self.convert_data(gyro_raw_data[4], gyro_raw_data[5])

            

                # Convert to correct units
            """
            # Apply low-pass filter to raw values
            accel_x = self.low_pass_filter(accel_x, prev_accel_x)
            accel_y = self.low_pass_filter(accel_y, prev_accel_y)
            accel_z = self.low_pass_filter(accel_z, prev_accel_z)

            gyro_x = self.low_pass_filter(gyro_x, prev_gyro_x)
            gyro_y = self.low_pass_filter(gyro_y, prev_gyro_y)
            gyro_z = self.low_pass_filter(gyro_z, prev_gyro_z)

            # Update previous values for next iteration
            prev_accel_x, prev_accel_y, prev_accel_z = accel_x, accel_y, accel_z
            prev_gyro_x, prev_gyro_y, prev_gyro_z = gyro_x, gyro_y, gyro_z

            accel_x = accel_x / ACCEL_SENSITIVITY * 9.81  # Convert to m/s²
            accel_y = accel_y / ACCEL_SENSITIVITY * 9.81
            accel_z = accel_z / ACCEL_SENSITIVITY * 9.81

            gyro_x = gyro_x / GYRO_SENSITIVITY * (np.pi / 180.0)  # Convert to rad/s
            gyro_y = gyro_y / GYRO_SENSITIVITY * (np.pi / 180.0)
            gyro_z = gyro_z / GYRO_SENSITIVITY * (np.pi / 180.0)
            """

            #convert to acceleration in g and gyro dps
            accel_x = (accel_x/(2.0**15.0))*ACCEL_SENSITIVITY
            accel_y = (accel_y/(2.0**15.0))*ACCEL_SENSITIVITY
            accel_z = (accel_z/(2.0**15.0))*ACCEL_SENSITIVITY

            gyro_x = (gyro_x/(2.0**15.0))*GYRO_SENSITIVITY
            gyro_y = (gyro_y/(2.0**15.0))*GYRO_SENSITIVITY
            gyro_z = (gyro_z/(2.0**15.0))*GYRO_SENSITIVITY

            accel_data.append([accel_x, accel_y, accel_z])
            gyro_data.append([gyro_x, gyro_y, gyro_z])

            
        
        accel_data = np.array(accel_data)
        gyro_data = np.array(gyro_data)

        # Accelerometer calibration (slope and offset calculation using linear regression)
        accel_mean = np.mean(accel_data, axis=0)
        accel_min = np.min(accel_data, axis=0)
        accel_max = np.max(accel_data, axis=0)

        accel_slope = np.ones(3)  # Slope should generally be near 1 for accelerometer
        accel_offset = accel_mean  # Use the mean as the offset

        # Gyroscope calibration (offset calculation)
        gyro_offset = np.mean(gyro_data, axis=0)

        # Store calibration parameters
        calibration_data[key]["accel"]["slope"] = accel_slope
        calibration_data[key]["accel"]["offset"] = accel_offset
        calibration_data[key]["gyro"]["offset"] = gyro_offset

        # Log the standard deviation for diagnostic purposes
        # You can also compute standard deviation and discard high variance data
        accel_std = np.std(accel_data, axis=0)
        gyro_std = np.std(gyro_data, axis=0)
        self.get_logger().info(f"Accel standard deviation: {accel_std}, Gyro standard deviation: {gyro_std}")

        self.get_logger().info(f"Calibration completed for {key}. Accel offsets: {accel_offset}, Gyro offsets: {gyro_offset}")

    def low_pass_filter_raw(data, alpha=0.5):
        filtered_data = [data[0]]  # Initialize with the first value
        for i in range(1, len(data)):
            filtered_data.append(alpha * data[i] + (1 - alpha) * filtered_data[i - 1])
        return np.array(filtered_data)
    def convert_data(self, high_byte, low_byte):
        """Converts high and low bytes into a signed integer"""
        value = (high_byte << 8) | low_byte
        if value > 32767:
            value -= 65536
        return value
    
    def low_pass_filter(current_value, previous_value, alpha=0.5):
        """Applies a low-pass filter to smooth raw sensor data."""
        return alpha * current_value + (1 - alpha) * previous_value
    def read_sensor_data(self, address, calibration_key,prev):
        """Reads the raw sensor data from the MPU9250"""
        # Read accelerometer and gyroscope data
        accel_data = self.bus.read_i2c_block_data(address, 0x3B, 6)
        gyro_data = self.bus.read_i2c_block_data(address, 0x43, 6)

        # Process accelerometer data
        accel_x = self.convert_data(accel_data[0], accel_data[1])
        accel_y = self.convert_data(accel_data[2], accel_data[3])
        accel_z = self.convert_data(accel_data[4], accel_data[5])

        # Process gyroscope data
        gyro_x = self.convert_data(gyro_data[0], gyro_data[1])
        gyro_y = self.convert_data(gyro_data[2], gyro_data[3])
        gyro_z = self.convert_data(gyro_data[4], gyro_data[5])
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
        accel_x = ( calibration_data[calibration_key]["accel"]["offset"][0]) +   accel_x* calibration_data[calibration_key]["accel"]["slope"][0]
        accel_y = (  calibration_data[calibration_key]["accel"]["offset"][1]) +  accel_y * calibration_data[calibration_key]["accel"]["slope"][1]
        accel_z = (  calibration_data[calibration_key]["accel"]["offset"][2])  + accel_z * calibration_data[calibration_key]["accel"]["slope"][2]
        gyro_x -= calibration_data[calibration_key]["gyro"]["offset"][0]
        gyro_y -= calibration_data[calibration_key]["gyro"]["offset"][1]
        gyro_z -= calibration_data[calibration_key]["gyro"]["offset"][2]


        
        

        return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

    
    def timer_callback(self):
        msg = Mpu()
        try:
            self.current_time = time.time()
            if self.current_time - self.calibrationTime > 600:
                self.calibrate_mpu(mpu9250_address,10000,'mpu1')
                self.calibrate_mpu(mpu9250_address_2,10000,'mpu2')
            # Read accelerometer data
            key = 'mpu1'
            prev = [self.prev_accel_x, self.prev_accel_y, self.prev_accel_z, self.prev_gyro_x, self.prev_gyro_y, self.prev_gyro_z]
            accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z = self.read_sensor_data(mpu9250_address,key ,prev)
            self.prev_accel_x, self.prev_accel_y, self.prev_accel_z = accel_x, accel_y, accel_z
            self.prev_gyro_x, self.prev_gyro_y, self.prev_gyro_z = gyro_x, gyro_y, gyro_z
            # Read, calibrate, and convert gyroscope data to dps
            key = 'mpu2'
            prev = [self.prev_accel_x2, self.prev_accel_y2, self.prev_accel_z2, self.prev_gyro_x2, self.prev_gyro_y2, self.prev_gyro_z2]
            accel_x_2 ,accel_y_2,accel_z_2,gyro_x_2,gyro_y_2,gyro_z_2 = self.read_sensor_data(mpu9250_address, key,prev )
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
