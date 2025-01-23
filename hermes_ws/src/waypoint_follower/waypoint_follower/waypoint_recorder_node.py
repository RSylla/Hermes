import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import time

#!/usr/bin/env python3


class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')
        
        # Create subscriber for GPS data
        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10)
        
        self.last_save_time = time.time()
        self.save_interval = 0.5  # Save every 0.5 seconds
        self.filename = 'src/waypoint_follower/waypoint_follower/waypoints.txt'
        self.get_logger().info('Waypoint recorder started. Saving to waypoints.txt')

    def gps_callback(self, msg):
        current_time = time.time()
        
        # Save coordinates every 0.5 seconds
        if current_time - self.last_save_time >= self.save_interval:
            try:
                with open(self.filename, 'a') as f:
                    f.write(f"{msg.latitude},{msg.longitude}\n")
                self.get_logger().info(f'Saved waypoint: {msg.latitude}, {msg.longitude}')
                self.last_save_time = current_time
            except Exception as e:
                self.get_logger().error(f'Error saving waypoint: {e}')

def main(args=None):
    rclpy.init(args=args)
    waypoint_recorder = WaypointRecorder()
    
    try:
        rclpy.spin(waypoint_recorder)
    except KeyboardInterrupt:
        pass
    finally:
        waypoint_recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()