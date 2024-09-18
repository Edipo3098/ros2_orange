import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Mpu, COGframe
from time import time
import numpy as np

from filterpy.kalman import KalmanFilter

class IMUKalmanFilter:
    def __init__(self, dt):
        self.kf = KalmanFilter(dim_x=9, dim_z=6)  # State vector has 9 dimensions, measurements have 6
        self.dt = dt
        
        # State transition matrix (F)
        self.kf.F = np.eye(9)
        self.kf.F[0, 3] = self.dt  # x += vx * dt
        self.kf.F[1, 4] = self.dt  # y += vy * dt
        self.kf.F[2, 5] = self.dt  # z += vz * dt
        
        # Measurement matrix (H)
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
        self.kf.R = np.diag([0.5, 0.5, 0.5, 0.1, 0.1, 0.1])
        
        # Covariance matrix (P)
        self.kf.P = np.eye(9) * 0.01  # Low uncertainty in initial position and velocity
    
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
        self.subscription_mpu = self.create_subscription(
            Mpu,
            'mpu_data',
            self.listener_callback,
            10)
        self.subscription_mpu2 = self.create_subscription(
            Mpu,
            'mpu_data_2',
            self.listener_callback2,
            10)
        self.subscription_mpu  # prevent unused variable warning
        self.publishKalmanFrame = self.create_publisher(COGframe, 'kalman_cog_frame', 10)
        self.publishTrapezFrame = self.create_publisher(COGframe, 'trapez_cog_frame', 10)

        # Kalman filter for fusing data from both MPUs
        self.kf = IMUKalmanFilter(dt=0.01)
        # Buffers to hold the last measurements from each MPU
        self.mpu1_data = None
        self.mpu2_data = None


        # Store previous values for trapezoidal integration
        self.prev_time = time()
        self.prev_time2 = time()

        

        # For linear position integration (acceleration to velocity to position)
        self.prev_acx = 0.0
        self.prev_acy = 0.0
        self.prev_acz = 0.0
        self.prev_gx = 0.0
        self.prev_gy = 0.0
        self.prev_gz = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        # For angular position integration (angular velocity to angular position)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        

    def listener_callback(self, msg):
        """
        Callback function that processes incoming accelerometer and gyroscope data,
        and calculates the linear position (x, y, z) and angular position (roll, pitch, yaw)
        using trapezoidal integration.
        """
        
        self.mpu1_data = msg
        self.process_fusion()
        
    def listener_callback2(self, msg):
        """
        Callback function that processes incoming accelerometer and gyroscope data,d
        and calculates the linear position (x, y, z) and angular position (roll, pitch, yaw)
        using trapezoidal integration.
        """
        self.mpu2_data = msg
        self.process_fusion()
        
    def apply_deadzone(self,value, threshold=1e-3):
        """Apply deadzone to filter out small noise values."""
        return value if abs(value) > threshold else 0.0
    
    def remove_gravity(self,acx, acy, acz, roll, pitch, yaw):
        # Compute gravity vector in sensor's frame based on orientation
        gravity_vector = np.array([
            -9.81 * np.sin(pitch),  # Gravity in X direction based on pitch
            9.81 * np.sin(roll),    # Gravity in Y direction based on roll
            -9.81 * np.cos(pitch) * np.cos(roll)  # Gravity in Z direction based on both pitch and roll
        ])

        # Subtract or add gravity depending on sensor reading
        acx_corrected = acx - gravity_vector[0]
        acy_corrected = acy - gravity_vector[1]
        acz_corrected = acz - gravity_vector[2]

        return acx_corrected, acy_corrected, acz_corrected

    def compute_rotation_matrix(self, roll, pitch, yaw):
        # Compute the rotation matrix from roll, pitch, yaw
        cr = np.cos(roll)
        sr = np.sin(roll)
        cp = np.cos(pitch)
        sp = np.sin(pitch)
        cy = np.cos(yaw)
        sy = np.sin(yaw)

        # Rotation matrix
        R = np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp,     cp * sr,               cp * cr]
        ])
        return R
    def process_fusion(self):
        if self.mpu1_data is None or self.mpu2_data is None:
            return  # Wait until we have data from both MPUs

        current_time = time()
        dt = current_time - self.prev_time
        if dt < 1e-6:
            return  # Prevent instability from very small time steps

        self.kf.change_dt(dt)  # Ensure Kalman filter uses the same dt
        self.prev_time = current_time  # Update prev_time after dt calculation

        # Compute the average measurements from both MPUs (sensor fusion)
        avg_acx = self.apply_deadzone( (self.mpu1_data.acx + self.mpu2_data.acx) / 2)
        avg_acy = self.apply_deadzone((self.mpu1_data.acy + self.mpu2_data.acy) / 2)
        avg_acz = self.apply_deadzone((self.mpu1_data.acz + self.mpu2_data.acz) / 2)
        avg_gx = self.apply_deadzone((self.mpu1_data.gx + self.mpu2_data.gx) / 2)
        avg_gy = self.apply_deadzone((self.mpu1_data.gy + self.mpu2_data.gy) / 2)
        avg_gz = self.apply_deadzone((self.mpu1_data.gz + self.mpu2_data.gz) / 2)

        # Prepare the measurement vector
        measurements = np.array([avg_acx, avg_acy, avg_acz, avg_gx, avg_gy, avg_gz])

        # Update the Kalman filter with fused data
        self.kf.dt = dt
        self.kf.predict()
        # Use the Kalman filter to estimate the orientation (roll, pitch, yaw)
        _, _, (roll, pitch, yaw) = self.kf.get_state()
        # Remove gravity from accelerometer readings
        accel_raw = np.array([avg_acx, avg_acy, avg_acz])
        accel_corrected = self.remove_gravity(avg_acx,avg_acy, roll, avg_acz,pitch, yaw)

        # Prepare the corrected measurement vector
        #measurements = np.array([avg_acx,avg_acy, roll, avg_acz,pitch, yaw])

        # Update the Kalman filter with gravity-corrected data
        self.kf.change_dt(dt)
        self.kf.predict()
        self.kf.update(measurements)
        # Retrieve filtered state (position, velocity, orientation)
        
        pos, vel, orient = self.kf.get_state()
        alpha = 0.98  # Weight for the trapezoidal integration result
        beta = 1 - alpha  # Weight for the Kalman filter result

        
        msg = COGframe()
        msg.pos_x = float(pos[0])
        msg.pos_y = float(pos[1])
        msg.pos_z = float(pos[2])
        msg.roll = float(orient[0])
        msg.pitch = float(orient[1])
        msg.yaw = float(orient[2])
        
        self.publishKalmanFrame.publish(msg)
        self.get_logger().info(f"Kalman pose (X, Y, Z): {msg.pos_x}, {msg.pos_y}, {msg.pos_z}")
        self.get_logger().info(f"Kalman orientation (roll pitch yaw): {msg.roll }, {msg.pitch}, {msg.yaw}")
        self.calculatePose(vel[0], vel[1], vel[2], orient[0], orient[1], orient[2],dt)
        msg.pos_x= float(self.x)
        msg.pos_y = float(self.y)
        msg.pos_z = float(self.z)
        msg.roll = float(self.roll)
        msg.pitch = float(self.pitch)
        msg.yaw = float(self.yaw)
        
        
        self.get_logger().info(f"trapzo pose (X, Y, Z): {msg.pos_x}, {msg.pos_y}, {msg.pos_z}")
        self.get_logger().info(f"trapzo orientation (roll pitch yaw): {msg.roll }, {msg.pitch}, {msg.yaw}")
        self.publishTrapezFrame.publish(msg)
        

        
   

    def calculatePose(self,vx,vy,vz,gx,gy,gz,dt):
        
        if dt < 1e-6:  # Check for very small dt to avoid instability
            return

        # Linear position calculation (acceleration -> velocity -> position)
        self.vx = vx
        self.vy = vy
        self.vz = vz

        self.x += self.vx * dt  # Integrate velocity to get position
        self.y += self.vy * dt
        self.z += self.vz * dt
        # Update previous values for the next integration step
        
        # Angular position calculation (angular velocity -> angle)
        self.roll += self.trapezoidal_integral(self.prev_gx, gx, dt)  # Integrate angular velocity to get roll
        self.pitch += self.trapezoidal_integral(self.prev_gy, gy, dt)  # Integrate angular velocity to get pitch
        self.yaw += self.trapezoidal_integral(self.prev_gz, gz, dt)  # Integrate angular velocity to get yaw

        # Update previous values for next integration step
        
        self.prev_gx = gx
        self.prev_gy = gy
        self.prev_gz = gz
        

    def trapezoidal_integral(self, prev_value, current_value, dt):
        """
        Perform trapezoidal integration to compute the integral of the given values over time.
        """
        return (prev_value + current_value) / 2.0 * dt

def main(args=None):
    rclpy.init(args=args)

    cog_calculate = CalCOGFrame()

    rclpy.spin(cog_calculate)

    # Destroy the node explicitly
    cog_calculate.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
