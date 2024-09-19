import rclpy
from rclpy.node import Node
import smbus
import time
from robot_interfaces.msg import Mpu
import numpy as np
from sensor_msgs.msg import Imu

# MPU9250 constants
mpu9250_address = 0x68  # MPU9250 default I2C address
mpu9250_address_2 = 0x69  # Second MPU9250 I2C address AD0 high
PWR_MGMT_1 = 0x6B
ACCEL_SENSITIVITY = 16384.0  # +/- 2g sensitivity
GYRO_SENSITIVITY = 131.0  # +/- 250 dps sensitivity

# Calibration data for two MPU9250s
calibration_data = {
    "mpu1": {
        "accel": {"slope": [1.0, 1.0, 1.0], "offset": [0.0, 0.0, 0.0]},
        "gyro": {"offset": [0.0, 0.0, 0.0]}
    },
    "mpu2": {
        "accel": {"slope": [1.0, 1.0, 1.0], "offset": [0.0, 0.0, 0.0]},
        "gyro": {"offset": [0.0, 0.0, 0.0]}
    }
}

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('mpu_publisher')

        # Publishers for MPU and Imu data
        self.publisher_mpu1 = self.create_publisher(Mpu, 'mpu_data_1', 10)
        self.publisher_mpu2 = self.create_publisher(Mpu, 'mpu_data_2', 10)

        # I2C bus initialization
        self.bus = smbus.SMBus(1)  # Using I2C bus 1
        self.initialize_mpu(mpu9250_address)
        self.initialize_mpu(mpu9250_address_2)

        # Timer to read and publish data periodically
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        # Calibrate MPU on startup
        self.calibrate_mpu(mpu9250_address, "mpu1")
        self.calibrate_mpu(mpu9250_address_2, "mpu2")

    def initialize_mpu(self, address):
        """Initializes the MPU by waking it up from sleep mode"""
        self.bus.write_byte_data(address, PWR_MGMT_1, 0x00)
        time.sleep(0.1)

    def read_sensor_data(self, address, calibration_key):
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

        # Apply calibration
        accel_x = (accel_x - calibration_data[calibration_key]["accel"]["offset"][0]) * calibration_data[calibration_key]["accel"]["slope"][0]
        accel_y = (accel_y - calibration_data[calibration_key]["accel"]["offset"][1]) * calibration_data[calibration_key]["accel"]["slope"][1]
        accel_z = (accel_z - calibration_data[calibration_key]["accel"]["offset"][2]) * calibration_data[calibration_key]["accel"]["slope"][2]

        gyro_x -= calibration_data[calibration_key]["gyro"]["offset"][0]
        gyro_y -= calibration_data[calibration_key]["gyro"]["offset"][1]
        gyro_z -= calibration_data[calibration_key]["gyro"]["offset"][2]

        # Convert to correct units
        accel_x /= ACCEL_SENSITIVITY
        accel_y /= ACCEL_SENSITIVITY
        accel_z /= ACCEL_SENSITIVITY

        gyro_x /= GYRO_SENSITIVITY
        gyro_y /= GYRO_SENSITIVITY
        gyro_z /= GYRO_SENSITIVITY

        return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

    def convert_data(self, high_byte, low_byte):
        """Converts high and low bytes into a signed integer"""
        value = (high_byte << 8) | low_byte
        if value > 32767:
            value -= 65536
        return value

    def calibrate_mpu(self, address, calibration_key):
        """Calibrates the MPU by calculating offsets"""
        accel_offset = [0.0, 0.0, 0.0]
        gyro_offset = [0.0, 0.0, 0.0]

        # Take multiple readings and average them to find the offsets
        num_samples = 100
        for i in range(num_samples):
            accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.read_sensor_data(address, calibration_key)
            accel_offset[0] += accel_x
            accel_offset[1] += accel_y
            accel_offset[2] += accel_z
            gyro_offset[0] += gyro_x
            gyro_offset[1] += gyro_y
            gyro_offset[2] += gyro_z

        # Calculate average offsets
        accel_offset = [x / num_samples for x in accel_offset]
        gyro_offset = [x / num_samples for x in gyro_offset]

        # Store the offsets for calibration
        calibration_data[calibration_key]["accel"]["offset"] = accel_offset
        calibration_data[calibration_key]["gyro"]["offset"] = gyro_offset

        print(f"Calibration completed for {calibration_key}. Accel offsets: {accel_offset}, Gyro offsets: {gyro_offset}")

    def check_full_scale(self, address):
        """Checks if the sensor is configured for full scale"""
        accel_config = self.bus.read_byte_data(address, 0x1C)
        gyro_config = self.bus.read_byte_data(address, 0x1B)

        accel_full_scale = (accel_config >> 3) & 0x03
        gyro_full_scale = (gyro_config >> 3) & 0x03

        if accel_full_scale != 0 or gyro_full_scale != 0:
            print("Warning: Sensor is not configured for full scale (2g accel, 250 dps gyro).")

    def timer_callback(self):
        """Callback function to read sensor data and publish it"""
        # Read data from MPU1
        accel_x1, accel_y1, accel_z1, gyro_x1, gyro_y1, gyro_z1 = self.read_sensor_data(mpu9250_address, "mpu1")
        mpu_msg1 = Mpu()
        mpu_msg1.accel_x = accel_x1
        mpu_msg1.accel_y = accel_y1
        mpu_msg1.accel_z = accel_z1
        mpu_msg1.gyro_x = gyro_x1
        mpu_msg1.gyro_y = gyro_y1
        mpu_msg1.gyro_z = gyro_z1
        self.publisher_mpu1.publish(mpu_msg1)

        # Read data from MPU2
        accel_x2, accel_y2, accel_z2, gyro_x2, gyro_y2, gyro_z2 = self.read_sensor_data(mpu9250_address_2, "mpu2")
        mpu_msg2 = Mpu()
        mpu_msg2.accel_x = accel_x2
        mpu_msg2.accel_y = accel_y2
        mpu_msg2.accel_z = accel_z2
        mpu_msg2.gyro_x = gyro_x2
        mpu_msg2.gyro_y = gyro_y2
        mpu_msg2.gyro_z = gyro_z2
        self.publisher_mpu2.publish(mpu_msg2)

        # Optionally check full-scale settings
        self.check_full_scale(mpu9250_address)
        self.check_full_scale(mpu9250_address_2)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
