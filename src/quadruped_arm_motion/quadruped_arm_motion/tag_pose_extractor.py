# src/quadruped_arm_motion/tag_odom_from_tf.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, LookupException
from rclpy.time import Time
from rclpy.duration import Duration

class TagOdomFromTF(Node):
    def __init__(self):
        super().__init__('tag_odom_from_tf')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.declare_parameter('tag_ids', [0, 1])
        self.declare_parameter('base_frame', 'base_link')

        tag_ids = self.get_parameter('tag_ids').get_parameter_value().integer_array_value
        base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        self.pubs = {}
        for tid in tag_ids:
            topic = f'/tag_odom_{tid}'
            self.pubs[tid] = self.create_publisher(Odometry, topic, 10)

        self.create_timer(0.1, lambda: self.publish_tag_odom(base_frame, tag_ids))

    def publish_tag_odom(self, base_frame, tag_ids):
        now = Time()
        for tid in tag_ids:
            tag_frame = f'tag36h11:{tid}'
            try:
                tf_stamped = self.tf_buffer.lookup_transform(
                    base_frame, tag_frame, now, timeout=Duration(seconds=0.05))
            except LookupException:
                continue

            msg = Odometry()
            msg.header = tf_stamped.header
            msg.header.frame_id = base_frame
            msg.child_frame_id = f'tag36h11:{tid}'

            t = tf_stamped.transform.translation
            q = tf_stamped.transform.rotation
            msg.pose.pose.position.x = t.x
            msg.pose.pose.position.y = t.y
            msg.pose.pose.position.z = t.z
            msg.pose.pose.orientation = q
            # En TagOdomFromTF.publish_tag_odom, justo antes de self.pubs[tid].publish(msg):
            # Velocidad en base_link = 0 m/s y 0 rad/s
            msg.twist.twist.linear.x = 0.0
            msg.twist.twist.linear.y = 0.0
            msg.twist.twist.linear.z = 0.0
            msg.twist.twist.angular.x = 0.0
            msg.twist.twist.angular.y = 0.0
            msg.twist.twist.angular.z = 0.0

            # Covarianza de velocidades baja (confianza alta)
            msg.twist.covariance = [
                0.01,   0.0,    0.0,    0.0,    0.0,    0.0,
                0.0,    0.01,   0.0,    0.0,    0.0,    0.0,
                0.0,    0.0,    0.01,   0.0,    0.0,    0.0,
                0.0,    0.0,    0.0,    0.01,   0.0,    0.0,
                0.0,    0.0,    0.0,    0.0,    0.01,   0.0,
                0.0,    0.0,    0.0,    0.0,    0.0,    0.01
            ]


            
            

def main():
    rclpy.init()
    node = TagOdomFromTF()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()
