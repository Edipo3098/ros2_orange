import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Mpu, COGframe
from time import time
import numpy as np

class KalmanFilter:
    def __init__(self, dt, process_noise, measurement_noise, estimate_error):
        self.dt = dt  # Time step

        # State vector: [x, y, z, vx, vy, vz, roll, pitch, yaw]
        self.state = np.zeros(9)

        # Covariance matrix (uncertainty in the estimate)
        self.P = np.eye(9) * estimate_error

        self.Q = np.eye(9) * 1e-3  # Small process noise for stationary state
        self.R = np.diag([0.1, 0.1, 0.1, 0.05, 0.05, 0.05])  # Measurement noise tuned for accelerometer/gyroscope


        # Measurement matrix
        self.H = np.zeros((6, 9))
        self.H[0, 3] = 1  # acx to vx
        self.H[1, 4] = 1  # acy to vy
        self.H[2, 5] = 1  # acz to vz
        self.H[3, 6] = 1  # gx to roll
        self.H[4, 7] = 1  # gy to pitch
        self.H[5, 8] = 1  # gz to yaw

        # Identity matrix
        self.I = np.eye(9)

    def predict(self):
        # State transition matrix for constant velocity and angular velocity
        F = np.eye(9)
        F[0, 3] = self.dt  # x = x + vx * dt
        F[1, 4] = self.dt  # y = y + vy * dt
        F[2, 5] = self.dt  # z = z + vz * dt
        F[6, 6] = self.dt  # roll = roll + roll_rate * dt
        F[7, 7] = self.dt  # pitch = pitch + pitch_rate * dt
        F[8, 8] = self.dt  # yaw = yaw + yaw_rate * dt

        # Predict the next state (assuming constant velocity and angular velocity)
        self.state = F @ self.state
        self.P = F @ self.P @ F.T + self.Q  # Update uncertainty
    
    def update(self, measurements):
        # Calculate the Kalman gain
        S = self.H @ self.P @ self.H.T + self.R  # Residual covariance
        K = self.P @ self.H.T @ np.linalg.inv(S)  # Kalman gain

        # Measurement residual (difference between actual and predicted)
        y = measurements - (self.H @ self.state)

        # Update the state estimate
        self.state = self.state + K @ y

        # Update the uncertainty
        self.P = (self.I - K @ self.H) @ self.P

    def get_state(self):
        # Return the current state (position, velocity, orientation)
        return self.state[:3], self.state[3:6], self.state[6:9]  # Position, velocity, orientation

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
        self.kf = KalmanFilter(dt=0.01, process_noise=1e-5, measurement_noise=0.1, estimate_error=1.0)
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
        
    def apply_deadzone(self,value, threshold=1e-4):
        """Apply deadzone to filter out small noise values."""
        return value if abs(value) > threshold else 0.0
    def process_fusion(self):
        if self.mpu1_data is None or self.mpu2_data is None:
            return  # Wait until we have data from both MPUs

        current_time = time()
        dt = current_time - self.prev_time
        if dt < 1e-6:
            return  # Prevent instability from very small time steps
        
        # Compute the average measurements from both MPUs (sensor fusion)
        avg_acx = self.apply_deadzone( (self.mpu1_data.acx + self.mpu2_data.acx) / 2)
        avg_acy = self.apply_deadzone(self.mpu1_data.acy + self.mpu2_data.acy) / 2)
        avg_acz = self.apply_deadzone(self.mpu1_data.acz + self.mpu2_data.acz) / 2)
        avg_gx = self.apply_deadzone(self.mpu1_data.gx + self.mpu2_data.gx) / 2)
        avg_gy = self.apply_deadzone(self.mpu1_data.gy + self.mpu2_data.gy) / 2)
        avg_gz = self.apply_deadzone(self.mpu1_data.gz + self.mpu2_data.gz) / 2)

        # Prepare the measurement vector
        measurements = np.array([avg_acx, avg_acy, avg_acz, avg_gx, avg_gy, avg_gz])

        # Update the Kalman filter with fused data
        self.kf.dt = dt
        self.kf.predict()
        self.kf.update(measurements)

        # Retrieve filtered state (position, velocity, orientation)
        
        pos, vel, orient = self.kf.get_state()
        msg = COGframe()
        msg.pos_x = pos[0]
        msg.pos_y = pos[1]
        msg.pos_z = pos[2]
        msg.roll = orient[0]
        msg.pitch = orient[1]
        msg.yaw = orient[2]
        self.prev_time = current_time
        self.publishKalmanFrame.publish(msg)
        self.get_logger().info(f"Kalman pose (X, Y, Z): {pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}")
        self.get_logger().info(f"Kalman orientation (roll pitch yaw): {orient[0]:.2f}, {orient[1]:.2f}, {orient[2]:.2f}")
        self.calculatePose(vel[0], vel[1], vel[2], pos[0], pos[1], pos[2])
        msg.pos_x= self.x
        msg.pos_y = self.y
        msg.pos_z = self.z
        msg.roll = self.roll
        msg.pitch = self.pitch
        msg.yaw = self.yaw
        self.prev_time = current_time
        self.get_logger().info(f"Trapezoidal pose (X, Y, Z): {self.x}, {self.y}, {self.z}")
        self.get_logger().info(f"Trapezoidal orientation (roll pitch yaw): {self.roll}, {self.pitch}, {self.yaw}")
        self.publishTrapezFrame.publish(msg)
        

        
   

    def calculatePose(self,vx,vy,vz,gx,gy,gz):
        current_time = time()
        dt = current_time - self.prev_time  # Time difference since the last message

        if dt < 1e-6:  # Check for very small dt to avoid instability
            return

        # Linear position calculation (acceleration -> velocity -> position)
        self.vx = vx
        self.vy = vy
        self.vz = vz

        self.x += self.vx * dt  # Integrate velocity to get position
        self.y += self.vy * dt
        self.z += self.vz * dt

        # Angular position calculation (angular velocity -> angle)
        self.roll += self.trapezoidal_integral(self.prev_gx, gx, dt)  # Integrate angular velocity to get roll
        self.pitch += self.trapezoidal_integral(self.prev_gy, gy, dt)  # Integrate angular velocity to get pitch
        self.yaw += self.trapezoidal_integral(self.prev_gz, gz, dt)  # Integrate angular velocity to get yaw

        # Update previous values for next integration step
        self.prev_time = current_time
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
