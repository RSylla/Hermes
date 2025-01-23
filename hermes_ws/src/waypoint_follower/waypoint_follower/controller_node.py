#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
import math
import time

# ------------------
# Simple PID Class
# ------------------
class PIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, dt=0.1, max_output=1.0, min_output=-1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.max_output = max_output
        self.min_output = min_output

        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error):
        # Proportional
        p_term = self.kp * error
        
        # Integral
        self.integral += error * self.dt
        i_term = self.ki * self.integral
        
        # Derivative
        d_error = (error - self.prev_error) / self.dt
        d_term = self.kd * d_error
        
        # PID output
        output = p_term + i_term + d_term
        
        # Clamp output
        if output > self.max_output:
            output = self.max_output
        elif output < self.min_output:
            output = self.min_output
        
        # Store for next iteration
        self.prev_error = error
        
        return output

# ------------------
# Main ROS Node
# ------------------
class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # --- Load waypoints from file ---
        self.waypoints = self.load_waypoints(
            'src/waypoint_follower/waypoint_follower/waypoints.txt')
        self.current_waypoint_idx = 0

        # Reference lat/lon for approximate local coords
        # (e.g., first waypoint or home location)
        if len(self.waypoints) > 0:
            self.lat_ref, self.lon_ref = self.waypoints[0]
        else:
            self.get_logger().error("No waypoints found! Exiting.")
            rclpy.shutdown()
            return

        # Setup subscription to GPS
        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )

        # Setup publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_out', 10)

        # PID controllers
        # Example: For heading error
        self.heading_pid = PIDController(kp=1.5, ki=0.0, kd=0.0, dt=0.1,
                                         max_output=0.5, min_output=-0.5)

        # Timer to publish commands at 10 Hz
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # Current rover position and heading
        self.current_lat = None
        self.current_lon = None
        self.current_heading = 0.0  # Ideally from IMU, but we’ll approximate

        # Store previous lat/lon to compute heading if no IMU is available
        self.prev_lat = None
        self.prev_lon = None
        self.last_time = time.time()

        self.get_logger().info("Waypoint Follower node initialized.")

    def load_waypoints(self, filename):
        waypoints = []
        try:
            with open(filename, 'r') as f:
                for line in f:
                    lat_str, lon_str = line.strip().split(',')
                    lat = float(lat_str)
                    lon = float(lon_str)
                    waypoints.append((lat, lon))
        except Exception as e:
            self.get_logger().error(f"Error loading waypoints: {e}")
        return waypoints

    def gps_callback(self, msg: NavSatFix):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

        # Approximate heading from previous GPS position if no IMU
        if self.prev_lat is not None and self.prev_lon is not None:
            dx, dy = self.latlon_to_xy(self.current_lat, self.current_lon,
                                       self.prev_lat, self.prev_lon)
            # If dx, dy are near zero, heading remains the same
            if abs(dx) > 0.5 or abs(dy) > 0.5:
                # Heading = angle in [−pi, pi]
                self.current_heading = math.atan2(dy, dx)

        self.prev_lat = self.current_lat
        self.prev_lon = self.current_lon

    def control_loop(self):
        # If we don't have a valid GPS fix yet, do nothing
        if self.current_lat is None or self.current_lon is None:
            return

        # Check if we've reached all waypoints
        if self.current_waypoint_idx >= len(self.waypoints):
            # Stop the rover
            self.publish_cmd_vel(0.0, 0.0)
            self.get_logger().info("All waypoints reached. Stopping.")
            return

        # Current target waypoint
        wp_lat, wp_lon = self.waypoints[self.current_waypoint_idx]

        # Convert current and waypoint lat/lon to local XY
        x_r, y_r = self.latlon_to_xy(self.current_lat, self.current_lon,
                                     self.lat_ref, self.lon_ref)
        x_w, y_w = self.latlon_to_xy(wp_lat, wp_lon, self.lat_ref, self.lon_ref)

        # Distance to waypoint
        dist = math.sqrt((x_w - x_r)**2 + (y_w - y_r)**2)

        # If close enough to current waypoint, move on
        if dist < 0.3:  # your chosen threshold
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_idx}. Moving to next.")
            self.current_waypoint_idx += 1
            return

        # Desired heading
        desired_heading = math.atan2((y_w - y_r), (x_w - x_r))

        # Heading error in [−pi, pi]
        heading_error = self.angle_normalize(desired_heading - self.current_heading)

        # Update heading PID to get angular speed
        angular_z = 0.3 * self.heading_pid.update(heading_error)

        # Simple logic for linear speed:
        # - Go at a constant speed if far from waypoint.
        # - You can also use a separate PID if you want more sophisticated behavior.
        linear_x = 0.5  # m/s, tune as needed

        # Publish cmd_vel
        self.publish_cmd_vel(linear_x, angular_z)

    def publish_cmd_vel(self, lin, ang):
        twist = Twist()
        twist.linear.x = lin
        twist.angular.z = ang
        self.cmd_vel_pub.publish(twist)

    def latlon_to_xy(self, lat, lon, lat0, lon0):
        """
        Very rough conversion from lat/lon to local x,y using a 'flat-Earth' approximation.
        (For small distances only!)
        """
        # Constants for approximation
        meters_per_deg_lat = 111320.0
        # Adjust for the reference latitude
        meters_per_deg_lon = 111320.0 * math.cos(math.radians(lat0))

        dx = (lon - lon0) * meters_per_deg_lon
        dy = (lat - lat0) * meters_per_deg_lat
        return dx, dy

    def angle_normalize(self, angle):
        # Normalize angle to [−pi, pi]
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
