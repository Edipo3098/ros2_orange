import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Mpu, COGframe
from time import time
import numpy as np
from filterpy.kalman import KalmanFilter

"""
Receive data from two MPU6050 sensors and estimate the center of gravity (COG) of the robot.
Data is i m/s2 and º/s.
Data is fused using a Kalman filter 
trapezoidal integration uses to calculate X and .
"""

class IMUKalmanFilter:
    def __init__(self, dt):
        self.kf = KalmanFilter(dim_x=9, dim_z=6)  # 9 state variables, 6 measurements
        self.dt = dt
        # Gyroscope noise variance (º/s)^2
        self.gyro_rms_noise = 0.1  # º/s-rms
        self.gyro_variance = self.gyro_rms_noise ** 2  # Variance for gx, gy, gz

        # Accelerometer noise variance (m/s^2)^2
        self.acc_rms_noise = 0.008 * 9.81  # Convert from g to m/s^2
        self.acc_variance = self.acc_rms_noise ** 2  # Variance for acx, acy, a
        # State transition matrix (F)
        self.kf.F = np.eye(9)
        # Position is updated by velocity over time dt
        self.kf.F[0, 3] = self.dt  # x += vx * dt
        self.kf.F[1, 4] = self.dt  # y += vy * dt
        self.kf.F[2, 5] = self.dt  # z += vz * dt
        
        # Measurement matrix (H) maps measurements (acceleration and angular velocity) to velocity and orientation
        self.kf.H = np.zeros((6, 9))
        self.kf.H[0, 3] = 1  # acx -> vx
        self.kf.H[1, 4] = 1  # acy -> vy
        self.kf.H[2, 5] = 1  # acz -> vz
        self.kf.H[3, 6] = 1  # gx -> roll
        self.kf.H[4, 7] = 1  # gy -> pitch
        self.kf.H[5, 8] = 1  # gz -> yaw
        
        # Process noise covariance matrix (Q)
        self.kf.Q = np.diag([1e-5, 1e-5, 1e-5, 1e-3, 1e-3, 1e-3, 1e-5, 1e-5, 1e-5])
        
        # Measurement noise covariance matrix (R)
        self.kf.R = np.diag([self.acc_variance, self.acc_variance, self.acc_variance, 
             self.gyro_variance, self.gyro_variance, self.gyro_variance])
        
        # Covariance matrix (P)
        self.kf.P = np.eye(9) * 1  # Initial uncertainty
    
    def predict(self):
        self.kf.predict()
    
    def update(self, measurements):
        self.kf.update(measurements)
    
    def change_dt(self, dt):
        self.dt = dt
        self.kf.F[0, 3] = dt
        self.kf.F[1, 4] = dt
        self.kf.F[2, 5] = dt
    
    def get_state(self):
        return self.kf.x[:3], self.kf.x[3:6], self.kf.x[6:9]  # Position, velocity, orientation

class CalCOGFrame(Node):

    def __init__(self):
        super().__init__('cog_calc')
        self.subscription_mpu = self.create_subscription(Mpu, 'mpu_data', self.listener_callback, 10)
        self.subscription_mpu2 = self.create_subscription(Mpu, 'mpu_data_2', self.listener_callback2, 10)
        
        self.publishKalmanFrame = self.create_publisher(COGframe, 'kalman_cog_frame', 10)
        self.publishTrapezFrame = self.create_publisher(COGframe, 'trapez_cog_frame', 10)

        # Kalman filter for fusing data from both MPUs
        self.kf = IMUKalmanFilter(dt=0.01)
        
        # Buffers to hold the last measurements from each MPU
        self.mpu1_data = None
        self.mpu2_data = None

        # Store previous values for trapezoidal integration
        self.prev_time = time()

        # Initialize state variables for trapezoidal integration
        self.vx, self.vy, self.vz = 0.0, 0.0, 0.0
        self.x, self.y, self.z = 0.0, 0.0, 0.0
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0
        self.prev_gx, self.prev_gy, self.prev_gz = 0.0, 0.0, 0.0

    def listener_callback(self, msg):
        self.mpu1_data = msg
        self.process_fusion()
        
    def listener_callback2(self, msg):
        self.mpu2_data = msg
        self.process_fusion()

    def apply_deadzone(self, value, threshold=1e-3):
        return value if abs(value) > threshold else 0.0
    
    def remove_gravity(self, acx, acy, acz, roll, pitch):
        # Compute gravity vector based on roll and pitch
        gravity_vector = np.array([
            -9.81 * np.sin(pitch),    # Gravity component in X direction
            9.81 * np.sin(roll),      # Gravity component in Y direction
            -9.81 * np.cos(pitch) * np.cos(roll)  # Gravity component in Z direction
        ])

        # Remove gravity from accelerometer readings
        acx -= gravity_vector[0]
        acy -= gravity_vector[1]
        acz -= gravity_vector[2]

        return acx, acy, acz

    def process_fusion(self):
        if self.mpu1_data is None or self.mpu2_data is None:
            return  # Wait until data from both MPUs is available

        current_time = time()
        dt = current_time - self.prev_time
        if dt < 1e-6:
            return  # Skip very small time steps to prevent instability

        self.kf.change_dt(dt)
        self.prev_time = current_time  # Update previous time

        # Average measurements from both MPUs (sensor fusion)
        avg_acx = self.apply_deadzone((self.mpu1_data.acx + self.mpu2_data.acx) / 2)
        avg_acy = self.apply_deadzone((self.mpu1_data.acy + self.mpu2_data.acy) / 2)
        avg_acz = self.apply_deadzone((self.mpu1_data.acz + self.mpu2_data.acz) / 2)
        avg_gx = self.apply_deadzone((self.mpu1_data.gx + self.mpu2_data.gx) / 2)
        avg_gy = self.apply_deadzone((self.mpu1_data.gy + self.mpu2_data.gy) / 2)
        avg_gz = self.apply_deadzone((self.mpu1_data.gz + self.mpu2_data.gz) / 2)

        # Correct acceleration for gravity
        avg_acx, avg_acy, avg_acz = self.remove_gravity(avg_acx, avg_acy, avg_acz, self.roll, self.pitch)

        # Update Kalman filter with the measurements (accelerations and angular velocities)
        measurements = np.array([avg_acx, avg_acy, avg_acz, avg_gx, avg_gy, avg_gz])
        self.kf.predict()  # Prediction step based on the current state
        self.kf.update(measurements)  # Update Kalman filter with new measurements

        # Retrieve filtered state (position, velocity, orientation)
        pos, vel, orient = self.kf.get_state()

        # Publish the Kalman filter output
        msg = COGframe()
        msg.pos_x, msg.pos_y, msg.pos_z = float(pos[0]), float(pos[1]), float(pos[2])
        msg.roll, msg.pitch, msg.yaw = float(orient[0]), float(orient[1]), float(orient[2])
        self.publishKalmanFrame.publish(msg)
        self.get_logger().info(f"Kalman pose (X, Y, Z): {msg.pos_x}, {msg.pos_y}, {msg.pos_z}")
        self.get_logger().info(f"Kalman orientation (roll pitch yaw): {msg.roll}, {msg.pitch}, {msg.yaw}")

        # Perform trapezoidal integration for velocity and position
        self.calculate_pose(vel[0], vel[1], vel[2], avg_gx, avg_gy, avg_gz, dt)

        # Publish the trapezoidal integration result
        msg.pos_x, msg.pos_y, msg.pos_z = float(self.x), float(self.y), float(self.z)
        msg.roll, msg.pitch, msg.yaw = float(self.roll), float(self.pitch), float(self.yaw)
        self.publishTrapezFrame.publish(msg)
        self.get_logger().info(f"Trapezoidal pose (X, Y, Z): {msg.pos_x}, {msg.pos_y}, {msg.pos_z}")
        self.get_logger().info(f"Trapezoidal orientation (roll pitch yaw): {msg.roll}, {msg.pitch}, {msg.yaw}")

    def calculate_pose(self, vx, vy, vz, gx, gy, gz, dt):
        if dt < 1e-6:
            return

        # Update velocity and position using trapezoidal integration
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z += self.vz * dt

        # Angular position (orientation) update using trapezoidal integration for gyroscope data
        self.roll += self.trapezoidal_integral(self.prev_gx, gx, dt)
        self.pitch += self.trapezoidal_integral(self.prev_gy, gy, dt)
        self.yaw += self.trapezoidal_integral(self.prev_gz, gz, dt)

        # Update previous angular velocity for next integration step
        self.prev_gx, self.prev_gy, self.prev_gz = gx, gy, gz

    def trapezoidal_integral(self, prev_value, current_value, dt):
        return (prev_value + current_value) / 2.0 * dt

def main(args=None):
    rclpy.init(args=args)
    cog_calculate = CalCOGFrame()
    rclpy.spin(cog_calculate)
    cog_calculate.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
