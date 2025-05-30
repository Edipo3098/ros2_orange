import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from rclpy.duration import Duration
import numpy as np
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64MultiArray
from tf2_msgs.msg import TFMessage
DEFAULT_COV = [
    0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.1
]

def transform_to_matrix(trans, rot):
    # Quaternion to rotation matrix
    q = rot
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),       2*(x*z + y*w),     trans.x],
        [2*(x*y + z*w),         1 - 2*(x*x + z*z),   2*(y*z - x*w),     trans.y],
        [2*(x*z - y*w),         2*(y*z + x*w),       1 - 2*(x*x + y*y), trans.z],
        [0, 0, 0, 1]
    ])
    return R

def matrix_to_pose(M):
    pos = M[:3, 3]
    R = M[:3, :3]
    # Rotation matrix to quaternion
    qw = np.sqrt(1 + np.trace(R)) / 2
    qx = (R[2,1] - R[1,2]) / (4*qw)
    qy = (R[0,2] - R[2,0]) / (4*qw)
    qz = (R[1,0] - R[0,1]) / (4*qw)
    return pos, Quaternion(x=qx, y=qy, z=qz, w=qw)

class TagOdomFromTF(Node):
    def __init__(self):
        super().__init__('tag_odom_from_tf')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.declare_parameter('tag_ids', [0, 1])
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('covariance', DEFAULT_COV)
        self.declare_parameter('frequency', 30.0)

        tag_ids = self.get_parameter('tag_ids').get_parameter_value().integer_array_value
        base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.covariance = self.get_parameter('covariance').get_parameter_value().double_array_value
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value

        self.pubs = {}
        for tid in tag_ids:
            topic = f'/tag_odom_{tid}'
            self.pubs[tid] = self.create_publisher(Odometry, topic, 10)

        self.base_frame = base_frame
        self.tag_ids = tag_ids

        self.create_timer(1.0 / self.frequency, self.publish_all_tag_odom)
        self.publisher_ = self.create_publisher(Float64MultiArray, '/tag36h11_5_pose', 10)
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )
        self.last_stamp = None

    def publish_all_tag_odom(self):
        for tid in self.tag_ids:
            try:
                # --- Compose: map->tag_fixed->tag->camera_link->base_link ---
                tf_map_tag_fixed = self.tf_buffer.lookup_transform(
                    'map', f'tag36h11:{tid}_fixed', rclpy.time.Time(), timeout=Duration(seconds=0.1))
                tf_tag_fixed_tag = self.tf_buffer.lookup_transform(
                    f'tag36h11:{tid}_fixed', f'tag36h11:{tid}', rclpy.time.Time(), timeout=Duration(seconds=0.1))
                tf_tag_camera = self.tf_buffer.lookup_transform(
                    f'tag36h11:{tid}', 'camera_link', rclpy.time.Time(), timeout=Duration(seconds=0.1))
                tf_camera_base = self.tf_buffer.lookup_transform(
                    'camera_link', self.base_frame, rclpy.time.Time(), timeout=Duration(seconds=0.1))

                # Convert all transforms to matrices
                T1 = transform_to_matrix(tf_map_tag_fixed.transform.translation, tf_map_tag_fixed.transform.rotation)
                T2 = transform_to_matrix(tf_tag_fixed_tag.transform.translation, tf_tag_fixed_tag.transform.rotation)
                T3 = transform_to_matrix(tf_tag_camera.transform.translation, tf_tag_camera.transform.rotation)
                T4 = transform_to_matrix(tf_camera_base.transform.translation, tf_camera_base.transform.rotation)

                # Total transform: map→tag_fixed→tag→camera_link→base_link
                T = T1 @ T2 @ T3 @ T4
                pos, q = matrix_to_pose(T)
            except Exception as e:
                self.get_logger().warn(f"Transform chain failed for tag {tid}: {e}")
                continue

            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.child_frame_id = self.base_frame

            msg.pose.pose.position.x = pos[0]
            msg.pose.pose.position.y = pos[1]
            msg.pose.pose.position.z = pos[2]
            msg.pose.pose.orientation = q

            msg.twist.twist.linear.x = 0.0
            msg.twist.twist.linear.y = 0.0
            msg.twist.twist.linear.z = 0.0
            msg.twist.twist.angular.x = 0.0
            msg.twist.twist.angular.y = 0.0
            msg.twist.twist.angular.z = 0.0

            msg.pose.covariance = list(self.covariance)
            msg.twist.covariance = [
                1e-6, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1e-6, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1e-6, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1e-3, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1e-3, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 1e-3
            ]

            self.pubs[tid].publish(msg)
    def tf_callback(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id == 'tag36h11:5':
                data = [
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                    transform.transform.rotation.w,
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z
                ]
                arr = Float64MultiArray()
                arr.data = data
                self.publisher_.publish(arr)
                self.get_logger().info(
                    f'Publicado /tag36h11_5_pose: x={data[0]:.3f}, y={data[1]:.3f}, z={data[2]:.3f}'
                )
def main():
    rclpy.init()
    node = TagOdomFromTF()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
