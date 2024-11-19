#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Quaternion, Vector3, Point
from std_srvs.srv import Trigger
import time

class SensorResetNode(Node):
    def __init__(self):
        super().__init__('sensor_reset_node')
        
        # Declare parameters
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('gps_topic', '/fix')
        self.declare_parameter('num_publishes', 5)
        
        # Get parameters
        self.odom_topic = self.get_parameter('odom_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.gps_topic = self.get_parameter('gps_topic').value
        self.num_publishes = self.get_parameter('num_publishes').value
        
        # Create publishers
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 10)
        self.gps_pub = self.create_publisher(NavSatFix, self.gps_topic, 10)
        
        # Create services
        self.reset_all_srv = self.create_service(
            Trigger, 'reset_sensors', self.reset_all_callback)
        self.reset_odom_srv = self.create_service(
            Trigger, 'reset_odometry', self.reset_odom_callback)
        self.reset_imu_srv = self.create_service(
            Trigger, 'reset_imu', self.reset_imu_callback)
        self.reset_gps_srv = self.create_service(
            Trigger, 'reset_gps', self.reset_gps_callback)
        
        self.get_logger().info('Sensor reset node initialized')

    def reset_odom(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        
        # Zero position
        odom_msg.pose.pose.position = Point(x=0.0, y=0.0, z=0.0)
        odom_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        # Zero velocities
        odom_msg.twist.twist.linear = Vector3(x=0.0, y=0.0, z=0.0)
        odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=0.0)
        
        # Publish multiple times
        for _ in range(self.num_publishes):
            self.odom_pub.publish(odom_msg)
            time.sleep(0.1)
        
        self.get_logger().info('Odometry reset completed')
        return True

    def reset_imu(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"
        
        # Zero orientation
        imu_msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        # Zero angular velocity and acceleration
        imu_msg.angular_velocity = Vector3(x=0.0, y=0.0, z=0.0)
        imu_msg.linear_acceleration = Vector3(x=0.0, y=0.0, z=0.0)
        
        # Publish multiple times
        for _ in range(self.num_publishes):
            self.imu_pub.publish(imu_msg)
            time.sleep(0.1)
        
        self.get_logger().info('IMU reset completed')
        return True

    def reset_gps(self):
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = "gps_link"
        
        # Set status
        gps_msg.status.status = -1  # No fix
        gps_msg.status.service = 1  # GPS
        
        # Zero position
        gps_msg.latitude = 0.0
        gps_msg.longitude = 0.0
        gps_msg.altitude = 0.0
        
        # Set covariance to zero
        gps_msg.position_covariance = [0.0] * 9
        gps_msg.position_covariance_type = 0  # Unknown
        
        # Publish multiple times
        for _ in range(self.num_publishes):
            self.gps_pub.publish(gps_msg)
            time.sleep(0.1)
        
        self.get_logger().info('GPS reset completed')
        return True

    def reset_all_callback(self, request, response):
        self.reset_odom()
        self.reset_imu()
        self.reset_gps()
        response.success = True
        response.message = 'All sensors reset successfully'
        return response

    def reset_odom_callback(self, request, response):
        success = self.reset_odom()
        response.success = success
        response.message = 'Odometry reset successfully' if success else 'Odometry reset failed'
        return response

    def reset_imu_callback(self, request, response):
        success = self.reset_imu()
        response.success = success
        response.message = 'IMU reset successfully' if success else 'IMU reset failed'
        return response

    def reset_gps_callback(self, request, response):
        success = self.reset_gps()
        response.success = success
        response.message = 'GPS reset successfully' if success else 'GPS reset failed'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SensorResetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
