#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
import math
import yaml
from pyproj import Transformer
import numpy as np

class StanleyController(Node):
    def __init__(self):
        super().__init__('stanley_controller')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('waypoints_file', ''),
                ('k_gain', 0.5),
                ('target_speed', 0.5),
                ('goal_threshold', 2.0),
                ('look_ahead', 1.0),
                ('utm_zone', 35),
                ('utm_band', 'N')
            ]
        )

        # Initialize transformer for GPS to UTM conversion
        self.transformer = Transformer.from_crs("EPSG:4326", "EPSG:32633", always_xy=True)
        
        # Load waypoints
        self.waypoints = self.load_waypoints()
        self.current_waypoint_index = 0
        
        # Initialize state variables
        self.current_x = None
        self.current_y = None
        self.current_heading = None
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.gps_sub = self.create_subscription(NavSatFix, 'fix', self.gps_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        # Control loop timer
        self.create_timer(0.1, self.control_loop)

    def load_waypoints(self):
        """Load and convert waypoints from YAML file"""
        try:
            waypoints_file = self.get_parameter('waypoints_file').value
            with open(waypoints_file, 'r') as f:
                data = yaml.safe_load(f)
                waypoints = []
                
                for wp in data['waypoints']:
                    x, y = self.transformer.transform(wp['longitude'], wp['latitude'])
                    waypoints.append({
                        'x': x,
                        'y': y,
                        'heading': wp['heading']
                    })
                return waypoints
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {str(e)}')
            return []

    def gps_callback(self, msg):
        """Convert GPS to UTM coordinates"""
        try:
            self.current_x, self.current_y = self.transformer.transform(
                msg.longitude, msg.latitude)
        except Exception as e:
            self.get_logger().error(f'GPS transform error: {str(e)}')

    def odom_callback(self, msg):
        """Extract heading from odometry"""
        quat = msg.pose.pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        self.current_heading = math.atan2(siny_cosp, cosy_cosp)

    def get_current_target(self):
        """Get current target waypoint"""
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            return None
        return self.waypoints[self.current_waypoint_index]

    def control_loop(self):
        """Main control loop implementing Stanley controller"""
        if None in (self.current_x, self.current_y, self.current_heading):
            return

        target = self.get_current_target()
        if target is None:
            self.cmd_vel_pub.publish(Twist())  # Stop if no target
            return

        try:
            # Calculate distance to target
            dx = target['x'] - self.current_x
            dy = target['y'] - self.current_y
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Check if waypoint is reached
            if distance < self.get_parameter('goal_threshold').value:
                self.current_waypoint_index += 1
                if self.current_waypoint_index >= len(self.waypoints):
                    self.get_logger().info('Final waypoint reached')
                    self.cmd_vel_pub.publish(Twist())
                return

            # Calculate path heading
            path_heading = math.atan2(dy, dx)
            
            # Heading error
            heading_error = path_heading - self.current_heading
            heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

            # Cross track error
            cross_track_error = distance * math.sin(heading_error)

            # Stanley control law
            k = self.get_parameter('k_gain').value
            v = self.get_parameter('target_speed').value
            
            # Calculate steering angle
            steering_angle = heading_error + math.atan2(k * cross_track_error, v)
            steering_angle = np.clip(steering_angle, -math.pi/4, math.pi/4)  # Limit steering angle

            # Create command velocity
            cmd_vel = Twist()
            cmd_vel.linear.x = v
            cmd_vel.angular.z = steering_angle
            
            # Publish command
            self.cmd_vel_pub.publish(cmd_vel)
            
            # Log status
            self.get_logger().debug(
                f'Distance: {distance:.2f}, Heading Error: {math.degrees(heading_error):.2f}°'
            )

        except Exception as e:
            self.get_logger().error(f'Control loop error: {str(e)}')
            self.cmd_vel_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = StanleyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()