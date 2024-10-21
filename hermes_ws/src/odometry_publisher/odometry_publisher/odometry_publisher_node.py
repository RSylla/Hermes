import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import tf2_ros
from std_msgs.msg import Int32
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

        # Initialize flag for first tick reception
        self.first_ticks_received = False

        # ROS2 Publishers, Subscribers, and Timers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(Int32, '/left_wheel_ticks', self.left_ticks_callback, 50)
        self.create_subscription(Int32, '/right_wheel_ticks', self.right_ticks_callback, 50)

        self.timer = self.create_timer(0.02, self.publish_odometry)  # 50 Hz

        self.ticks_left = 0
        self.ticks_right = 0

        # Velocity tracking
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Initialize time for velocity calculation
        self.previous_time = self.get_clock().now()

    def left_ticks_callback(self, msg):
        self.ticks_left = msg.data
        self.get_logger().debug(f"Left ticks received: {self.ticks_left}")
        self.check_first_ticks()

    def right_ticks_callback(self, msg):
        self.ticks_right = msg.data
        self.get_logger().debug(f"Right ticks received: {self.ticks_right}")
        self.check_first_ticks()

    def check_first_ticks(self):
        """Initialize last tick values when the first ticks are received."""
        if not self.first_ticks_received:
            self.last_ticks_left = self.ticks_left
            self.last_ticks_right = self.ticks_right
            self.first_ticks_received = True
            self.get_logger().debug("First ticks received. Initializing last tick values.")

    def calculate_odometry(self, dt):
        """Calculate the new position of the robot based on wheel tick counts."""
        if not self.first_ticks_received:
            self.get_logger().debug("Waiting for first ticks to be received.")
            return

        d_per_tick = math.pi * self.wheel_diameter / self.encoder_resolution
        d_left = (self.ticks_left - self.last_ticks_left) * d_per_tick
        d_right = (self.ticks_right - self.last_ticks_right) * d_per_tick
        self.last_ticks_left, self.last_ticks_right = self.ticks_left, self.ticks_right

        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_base

        # Update pose using the midpoint method for better accuracy
        dx = d_center * math.cos(self.theta + d_theta / 2.0)
        dy = d_center * math.sin(self.theta + d_theta / 2.0)
        self.x += dx
        self.y += dy
        self.theta += d_theta

        # Normalize theta to the range [-pi, pi]
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        # Compute velocities
        self.linear_velocity = d_center / dt
        self.angular_velocity = d_theta / dt

        self.get_logger().debug(f"Delta left: {d_left:.4f} m, Delta right: {d_right:.4f} m")
        self.get_logger().debug(f"Updated position: x={self.x:.4f}, y={self.y:.4f}, theta={self.theta:.4f} rad")
        self.get_logger().debug(f"Computed velocities: linear={self.linear_velocity:.4f} m/s, angular={self.angular_velocity:.4f} rad/s")

    def publish_odometry(self):
        """Calculate odometry and publish the Odometry message."""
        current_time = self.get_clock().now()
        dt = (current_time - self.previous_time).nanoseconds / 1e9  # Time interval in seconds
        self.previous_time = current_time

        if dt <= 0.0:
            self.get_logger().warn("Non-positive dt detected. Skipping odometry calculation.")
            return

        self.calculate_odometry(dt)

        # Create and populate the Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"  # Changed to base_footprint

        # Set the position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Set the orientation
        quat = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        # Set the velocity
        odom_msg.twist.twist.linear.x = self.linear_velocity
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.angular_velocity

        # Optionally, set covariance matrices
        odom_msg.pose.covariance = [0.0]*36
        odom_msg.twist.covariance = [0.0]*36

        # Publish the Odometry message
        self.odom_pub.publish(odom_msg)
        self.get_logger().debug("Odometry message published.")

        # Create and send the transform from odom to base_footprint
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().debug("Transform broadcasted from odom to base_footprint.")

def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = OdometryPublisherNode()
    try:
        rclpy.spin(odometry_publisher)
    except KeyboardInterrupt:
        odometry_publisher.get_logger().info("Odometry publisher node stopped cleanly")
    except Exception as e:
        odometry_publisher.get_logger().error(f"Exception in odometry publisher node: {e}")
    finally:
        odometry_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
