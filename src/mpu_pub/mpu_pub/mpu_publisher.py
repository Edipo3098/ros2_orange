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
# This node read data from MPU and publish data in raw values

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

mpu9250_address = 0x68  # MPU9250 default I2C address
mpu9250_address_2 = 0x69  # MPU9250 I2C address AD0 high
PWR_MGMT_1 = 0x6B
# Create an smbus object
i2c_bus = 2  # Assuming you want to use /dev/i2c-1


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
import numpy as np
from collections import deque

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('mpu_publisher')
        self.publisher_ = self.create_publisher(Mpu, 'mpu_data_1', 10)
        # Kalman Filters for each axis of accelerometer and gyroscope
        

        self.current_time = perf_counter()
        self.prevTime = perf_counter()
        self.dt = self.current_time - self.prevTime 

        self.current_time2 = perf_counter()
        self.prevTime2 = perf_counter()
        self.dt2 = self.current_time2 - self.prevTime2 

        self.msg = Mpu()
  

        # Publisher for Imu (standard message)
        self.imu_publisher_ = self.create_publisher(Imu, 'imu_data', 10)
        self.imu_publisher_second = self.create_publisher(Imu, 'imu_data_2', 10)
        self.i2c_bus = 2
        self.bus = smbus.SMBus(i2c_bus)
        # Then use self.bus.read_byte_data, etc. in the respective functions
        self.mpu_wake(mpu9250_address)
        self.mpu_wake(mpu9250_address_2)
        self.Check_communication(mpu9250_address)
        self.Check_communication(mpu9250_address_2)
        self.configure_sensor(mpu9250_address)
        self.configure_sensor(mpu9250_address_2)


        self.calibrationTime = time.time()
        self.current_time = time.time()
        

        self.check_full_scale(mpu9250_address)
       

        
        timer_period = 1/2000   # seconds 50Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.sendData = self.create_timer(timer_period, self.send_data)
        self.timer2 = self.create_timer(1, self.timer_callback2)
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

        """
        # Set the DLPF to 10 Hz (DLPF_CFG = 0b101) in the CONFIG register
        self.bus.write_byte_data(address, CONFIG, 0x05)
        time.sleep(0.1)
        """
        # Set the DLPF to allow higher frequency data through (DLPF_CFG = 0b000, for 260 Hz bandwidth)
        self.bus.write_byte_data(address, CONFIG, 0x00)  # Set the DLPF to a higher bandwidth configuration
        time.sleep(0.1)
        # Set the sample rate to 100 Hz (SMPLRT_DIV = 9)
        """
        sample_rate_div = 9  # 100 Hz sample rate
        self.bus.write_byte_data(address, SMPLRT_DIV, sample_rate_div)
        time.sleep(0.1)
        """
        # Set the sample rate to 4000 Hz (SMPLRT_DIV = 1 for 4000 Hz output rate)
        sample_rate_div = 1  # High sample rate
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
    
   
    
    def convert_data(self, high_byte, low_byte):
        """Converts high and low bytes into a signed integer"""
        value = (high_byte << 8) | low_byte
        if value > 32767:
            value -= 65536
        return value
    
    def low_pass_filter(self, current_value, previous_value, alpha=None):
        if alpha is None:
            alpha = self.dynamic_alpha_calculation(current_value, previous_value)
        return alpha * current_value + (1 - alpha) * previous_value

    def read_sensor_data(self, address, calibration_key,prev):
        """Reads the raw sensor data from the MPU9250"""
        # Read accelerometer and gyroscope data
        try:
            accel_data_error = self.bus.read_i2c_block_data(address, 0x3B, 6)
            gyro_data_error = self.bus.read_i2c_block_data(address, 0x43, 6)
        except Exception as e:
            #self.get_logger().info(f'Error in read_sensor_data: {e}')
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

        
       


        
        

        return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

    
    def timer_callback(self):
        self.current_time = perf_counter()
        
        self.dt = self.current_time - self.prevTime 

        self.prevTime = perf_counter()
        try:
            
            """
            if self.current_time - self.calibrationTime > 600:
                self.calibrate_mpu(mpu9250_address,20000,'mpu1')
                self.calibrate_mpu(mpu9250_address_2,20000,'mpu2')
            """
            # Read accelerometer data
            key = 'mpu1'
            prev = [self.prev_accel_x, self.prev_accel_y, self.prev_accel_z, self.prev_gyro_x, self.prev_gyro_y, self.prev_gyro_z]
            accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.read_sensor_data(mpu9250_address, key, prev)
            
            # Apply adaptive calibration to adjust offsets in real-time
            #self.adaptive_calibration([accel_x, accel_y, accel_z], key)
            #self.adaptive_calibration2([accel_x, accel_y, accel_z], key) # Second calibration method to Z axis

            self.prev_accel_x, self.prev_accel_y, self.prev_accel_z = accel_x, accel_y, accel_z
            self.prev_gyro_x, self.prev_gyro_y, self.prev_gyro_z = gyro_x, gyro_y, gyro_z
             # Update previous values for next loop iteration
            

            # Repeat the same process for the second MPU
            key = 'mpu2'
            prev = [self.prev_accel_x2, self.prev_accel_y2, self.prev_accel_z2, self.prev_gyro_x2, self.prev_gyro_y2, self.prev_gyro_z2]
            accel_x_2, accel_y_2, accel_z_2, gyro_x_2, gyro_y_2, gyro_z_2 = self.read_sensor_data(mpu9250_address_2, key, prev)

            # Apply adaptive calibration to MPU2
            #self.adaptive_calibration([accel_x_2, accel_y_2, accel_z_2], key)
            #self.adaptive_calibration2([accel_x_2, accel_y_2, accel_z_2], key) # Second calibration method to Z axis

            # Update previous values for MPU2
            self.prev_accel_x2, self.prev_accel_y2, self.prev_accel_z2 = accel_x_2, accel_y_2, accel_z_2
            self.prev_gyro_x2, self.prev_gyro_y2, self.prev_gyro_z2 = gyro_x_2, gyro_y_2, gyro_z_2



            
            
            #self.get_logger().info(f"Accel_x: {accel_x}, Accel_y: {accel_y}, Accel_z: {accel_z}")
            #self.get_logger().info(f"Accel_x_2: {accel_x_2}, Accel_y_2: {accel_y_2}, Accel_z_2: {accel_z_2}")
            #self.get_logger().info(f"Accel_x_2: {accel_x_2}, Accel_y_2: {accel_y_2}, Accel_z_2: {accel_z_2}")

            #self.get_logger().info(f"Gyro_x: {gyro_x}, Gyro_y: {gyro_y}, Gyro_z: {gyro_z}")
            #self.get_logger().info(f"Gyro_x_2: {gyro_x_2}, Gyro_y_2: {gyro_y_2}, Gyro_z_2: {gyro_z_2}")

            

 # Publish Acceleration data in g
    # Publish data in dps
            
           
            
            #self.get_logger().info('is publishing')

        except KeyboardInterrupt:
            pass
            #self.get_logger().info('Exiting the system key')
        #finally:
            #self.get_logger().info('Exiting the system')
    def timer_callback2(self):
        self.get_logger().info(f"Accel_x: {self.prev_accel_x}, Accel_y: {self.prev_accel_y}, Accel_z: {self.prev_accel_z}")
        self.get_logger().info(f"Accel_x_2: {self.prev_accel_x2}, Accel_y_2: {self.prev_accel_y2}, Accel_z_2: {self.prev_accel_z2}")
        self.get_logger().info(f"delta time process  {self.dt }")
        self.get_logger().info(f"delta time sen data {self.dt2 }")
    def send_data(self):
        self.current_time2 = perf_counter()
        
        self.dt2 = self.current_time2 - self.prevTime2 

        self.prevTime2= perf_counter()
        
        self.msg.message = "EL mensaje es"
        self.msg.acx = float(self.prev_accel_x)
        self.msg.acy = float(self.prev_accel_y)
        self.msg.acz = float(self.prev_accel_z)
        self.msg.gx = float(self.prev_gyro_x)
        self.msg.gy = float(self.prev_gyro_y)
        self.msg.gz = float(self.prev_gyro_x)

        
        
        self.msg.acx2 = float(self.prev_accel_x2)
        self.msg.acy2 = float(self.prev_accel_y2)
        self.msg.acz2 = float(self.prev_accel_z2)
        self.msg.gx2 = float(self.prev_gyro_x2)
        self.msg.gy2 = float(self.prev_gyro_x2)
        self.msg.gz2 = float(self.prev_gyro_x2)
        # Convert to Imu message and publish
        #imu_msg_1 = self.convert_mpu_to_imu(msg)  # First sensor
        #imu_msg_2 = self.convert_mpu_to_imu(msg2)  # Second sensor

        #self.imu_publisher_.publish(imu_msg_1)
        #self.imu_publisher_second.publish(imu_msg_2)
        self.publisher_.publish(self.msg)

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
