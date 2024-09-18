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

# Define the I2C bus and MPU9250 address
i2c_bus = 0  # On Orange Pi, I2C-0 is commonly used
mpu9250_address = 0x68  # MPU9250 default I2C address
mpu9250_address_2 = 0x69  # MPU9250 I2C address AD0 high

# Create an smbus object
i2c_bus = 1  # Assuming you want to use /dev/i2c-1


# MPU9250 register addresses
MPU9250_WHO_AM_I = 0x75
MPU9250_ACCEL_XOUT_H = 0x3B
MPU9250_ACCEL_YOUT_H = 0x3D
MPU9250_ACCEL_ZOUT_H = 0x3F
MPU9250_GYRO_XOUT_H = 0x43
MPU9250_GYRO_YOUT_H = 0x45
MPU9250_GYRO_ZOUT_H = 0x47


accel_calibration_x = {
    'a_x': 0.99910324,
    'm': 0.05304036,
    'b': 0.0  # You can add the bias term if needed
}
accel_calibration_y = {
    'a_x': 0.99910324,
    'm_y': 0.05304036,
    'b': 0.0  # You can add the bias term if needed
}
accel_calibration_z = {
    'a_x': 0.99910324,
    'm_z': 0.05304036,
    'b': 0.0  # You can add the bias term if needed
}
gyro_calibration_x = {
    'a_x': 1.00009231,
    'm': 0.01032118,
    'b': 0.0  # You can add the bias term if needed
}
gyro_calibration_y = {
    'a_x': 1.00009231,
    'm': 0.01032118,
    'b': 0.0  # You can add the bias term if needed
}
gyro_calibration_z = {
    'a_x': 1.00009231,
    'm': 0.01032118,
    'b': 0.0  # You can add the bias term if needed
}

accel_calibration_x_2 = {
    'a_x': 0.99910324,
    'm_x': 0.05304036,
    'b': 0.0  # You can add the bias term if needed
}
accel_calibration_y_2 = {
    'a_x': 0.99910324,
    'm_y': 0.05304036,
    'b': 0.0  # You can add the bias term if needed
}
accel_calibration_z_2 = {
    'a_x': 0.99910324,
    'm_z': 0.05304036,
    'b': 0.0  # You can add the bias term if needed
}
gyro_calibration_x_2 = {
    'a_x': 1.00009231,
    'm': 0.01032118,
    'b': 0.0  # You can add the bias term if needed
}
gyro_calibration_y_2 = {
    'a_x': 1.00009231,
    'm': 0.01032118,
    'b': 0.0  # You can add the bias term if needed
}
gyro_calibration_z_2 = {
    'a_x': 1.00009231,
    'm': 0.01032118,
    'b': 0.0  # You can add the bias term if needed
}
# Constants for sensitivity values
ACCEL_SENSITIVITY = 16384.0  # LSB/g for +/- 2g range
GYRO_SENSITIVITY = 131.0  # LSB/dps for +/- 250 dps range

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('mpu_publisher')
        self.publisher_ = self.create_publisher(Mpu, 'mpu_data', 10)
        self.publisher_secondMPU = self.create_publisher(Mpu, 'mpu_data_2', 10)
        self.Check_communication(mpu9250_address)
        self.Check_communication(mpu9250_address_2)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def Check_communication(self,address):
        try:
            bus = smbus.SMBus(i2c_bus)
            who_am_i = bus.read_byte_data(address, MPU9250_WHO_AM_I)
            self.get_logger().info('I heard: "%s"' % hex(who_am_i))
        except Exception as e:
            self.get_logger().info(f'Error in communication: {e}')
        finally:
            try:
                bus.close()
            except:
                pass
    
    

    def read_sensor_data(self, addres,register, calibration_params, sensitivity,IsGyro):
        try:
            bus = smbus.SMBus(i2c_bus)
            high = bus.read_byte_data(addres, register)
            low = bus.read_byte_data(addres, register + 1)
            value = (high << 8) | low

            if value > 32767:
                value -= 65536
            if IsGyro:
                # Apply calibration
                calibrated_value = value- calibration_params['b']
            else:
                # Apply calibration
                calibrated_value = calibration_params['a_x'] * value * calibration_params['m_x'] + calibration_params['b']
            # Convert to physical units
            physical_value = calibrated_value / sensitivity

            return physical_value
        except Exception as e:
            self.get_logger().info(f'Error in read_sensor_data: {e}')
        finally:
            try:
                bus.close()
            except:
                pass
    

    def timer_callback(self):
        msg = Mpu()
        try:
            # Read accelerometer data
            accel_x = self.read_sensor_data(mpu9250_address,MPU9250_ACCEL_XOUT_H, accel_calibration_x, ACCEL_SENSITIVITY,False)
            accel_y = self.read_sensor_data(mpu9250_address,MPU9250_ACCEL_YOUT_H, accel_calibration_y, ACCEL_SENSITIVITY,False)
            accel_z = self.read_sensor_data(mpu9250_address,MPU9250_ACCEL_ZOUT_H, accel_calibration_z, ACCEL_SENSITIVITY,False)
            # Read, calibrate, and convert gyroscope data to dps
            gyro_x = self.read_sensor_data(mpu9250_address,MPU9250_GYRO_XOUT_H, gyro_calibration_x, GYRO_SENSITIVITY,True)
            gyro_y = self.read_sensor_data(mpu9250_address,MPU9250_GYRO_YOUT_H, gyro_calibration_y, GYRO_SENSITIVITY,True)
            gyro_z = self.read_sensor_data(mpu9250_address,MPU9250_GYRO_ZOUT_H, gyro_calibration_z, GYRO_SENSITIVITY,True)

            # Read data from the second sensor on the second bus
            accel_x_2 = self.read_sensor_data( mpu9250_address_2, MPU9250_ACCEL_XOUT_H, accel_calibration_x_2, ACCEL_SENSITIVITY,False)
            accel_y_2 = self.read_sensor_data( mpu9250_address_2, MPU9250_ACCEL_YOUT_H, accel_calibration_y_2, ACCEL_SENSITIVITY,False)
            accel_z_2 = self.read_sensor_data( mpu9250_address_2, MPU9250_ACCEL_ZOUT_H, accel_calibration_z_2, ACCEL_SENSITIVITY,False)

            gyro_x_2 = self.read_sensor_data( mpu9250_address_2, MPU9250_GYRO_XOUT_H, gyro_calibration_x_2, GYRO_SENSITIVITY,True)
            gyro_y_2 = self.read_sensor_data( mpu9250_address_2, MPU9250_GYRO_YOUT_H, gyro_calibration_y_2, GYRO_SENSITIVITY,True)
            gyro_z_2 = self.read_sensor_data( mpu9250_address_2, MPU9250_GYRO_ZOUT_H, gyro_calibration_z_2, GYRO_SENSITIVITY,True)


            # Pause for a short duration
            time.sleep(0.1)
            msg.message = "EL mensaje es"
            msg.acx = accel_x
            msg.acy = accel_y
            msg.acz = accel_z
            msg.gx = gyro_x
            msg.gy = gyro_y
            msg.gz = gyro_z

            msg2 = Mpu()
            msg2.message = "EL mensaje es"
            msg2.acx = accel_x_2
            msg2.acy = accel_y_2
            msg2.acz = accel_z_2
            msg2.gx = gyro_x_2
            msg2.gy = gyro_y_2
            msg2.gz = gyro_z_2
            self.publisher_.publish(msg)
            self.publisher_secondMPU.publish(msg2)
            self.get_logger().info('is publishing')

        except KeyboardInterrupt:
            self.get_logger().info('Exiting the system key')
        finally:
            self.get_logger().info('Exiting the system')



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
