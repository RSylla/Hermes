import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import tf2_ros
from std_msgs.msg import Int16
from tf_transformations import quaternion_from_euler 

class OdometryPublisherNode(Node):
    def __init__(self):
        super().__init__('odometry_publisher')

        # Robot parameters
        self.wheel_diameter = 0.260  # meters
        self.encoder_resolution = 90  # ticks per revolution
        self.wheel_base = 0.53  # meters between the wheels

        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_ticks_left = 0
        self.last_ticks_right = 0

        # ROS2 Publishers, Subscribers, and Timers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(Int16, '/left_wheel_ticks', self.left_ticks_callback, 50)
        self.create_subscription(Int16, '/right_wheel_ticks', self.right_ticks_callback, 50)

        self.timer = self.create_timer(0.1, self.publish_odometry)  # 10 Hz

        self.ticks_left = 0
        self.ticks_right = 0

    def left_ticks_callback(self, msg):
        self.ticks_left = msg.data

    def right_ticks_callback(self, msg):
        self.ticks_right = msg.data

    def calculate_odometry(self):
        """Calculate the new position of the robot based on wheel tick counts."""
        d_per_tick = math.pi * self.wheel_diameter / self.encoder_resolution
        d_left = (self.ticks_left - self.last_ticks_left) * d_per_tick
        d_right = (self.ticks_right - self.last_ticks_right) * d_per_tick
        self.last_ticks_left, self.last_ticks_right = self.ticks_left, self.ticks_right

        d_center = (d_left + d_right) / 2
        d_theta = (d_right - d_left) / self.wheel_base

        self.theta += d_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

    def publish_odometry(self):
        """Calculate odometry and publish the Odometry message."""
        self.calculate_odometry()

        # Create and publish the Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        quat = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        self.odom_pub.publish(odom_msg)

        # Broadcast the transform
        transform = TransformStamped()
        transform.header.stamp = odom_msg.header.stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = OdometryPublisherNode()
    rclpy.spin(odometry_publisher)
    odometry_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

