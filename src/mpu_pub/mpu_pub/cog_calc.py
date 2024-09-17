import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Mpu, COGframe
from time import time

class CalCOGFrame(Node):

    def __init__(self):
        super().__init__('mpu_subscriber')
        self.subscription_mpu = self.create_subscription(
            Mpu,
            'mpu_data',
            self.listener_callback,
            10)
        self.subscription_mpu  # prevent unused variable warning
        self.publishCOGFrame = self.create_publisher(COGframe, 'cog_frame', 10)

        # Store previous values for trapezoidal integration
        self.prev_time = time()

        # For linear position integration (acceleration to velocity to position)
        self.prev_acx = 0.0
        self.prev_acy = 0.0
        self.prev_acz = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

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
        current_time = time()
        dt = current_time - self.prev_time  # Time difference since the last message

        if dt < 1e-6:  # Check for very small dt to avoid instability
            return

        # Linear position calculation (acceleration -> velocity -> position)
        self.vx += self.trapezoidal_integral(self.prev_acx, msg.acx, dt)  # Integrate acceleration to get velocity
        self.vy += self.trapezoidal_integral(self.prev_acy, msg.acy, dt)
        self.vz += self.trapezoidal_integral(self.prev_acz, msg.acz, dt)

        self.x += self.vx * dt  # Integrate velocity to get position
        self.y += self.vy * dt
        self.z += self.vz * dt

        # Angular position calculation (angular velocity -> angle)
        self.roll += self.trapezoidal_integral(self.prev_gx, msg.gx, dt)  # Integrate angular velocity to get roll
        self.pitch += self.trapezoidal_integral(self.prev_gy, msg.gy, dt)  # Integrate angular velocity to get pitch
        self.yaw += self.trapezoidal_integral(self.prev_gz, msg.gz, dt)  # Integrate angular velocity to get yaw

        # Update previous values for next integration step
        self.prev_time = current_time
        self.prev_acx = msg.acx
        self.prev_acy = msg.acy
        self.prev_acz = msg.acz
        self.prev_gx = msg.gx
        self.prev_gy = msg.gy
        self.prev_gz = msg.gz
        
        self.sendCOG_Frame()
        
    def sendCOG_Frame(self):
        msg = COGframe()
        msg.x = self.x
        msg.y = self.y
        msg.z = self.z
        msg.roll = self.roll
        msg.pitch = self.pitch
        msg.yaw = self.yaw
        self.publishCOGFrame.publish(msg)
        # For demonstration: print the calculated positions
        self.get_logger().info(f'Position: x={self.x}, y={self.y}, z={self.z}')
        self.get_logger().info(f'Orientation: roll={self.roll}, pitch={self.pitch}, yaw={self.yaw}')

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
