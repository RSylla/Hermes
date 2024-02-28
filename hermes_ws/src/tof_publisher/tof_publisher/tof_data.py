import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LaserScanSubscriber(Node):

    def __init__(self):
        super().__init__('laser_scan_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        range_start = 1
        range_end = 228
        sector_size = (range_end - range_start + 1) // 4  # Integer division for equal sectors

        for sector_num in range(4):
            sector_start = range_start + sector_num * sector_size
            sector_end = sector_start + sector_size - 1

            sector_values = msg.ranges[sector_start:sector_end + 1]
            if any(range < 1.0 and not math.isnan(range) for range in sector_values):
                self.get_logger().warn(f'Warning: Object too close in sector {sector_num + 1}')

def main(args=None):
    rclpy.init(args=args)
    laser_scan_subscriber = LaserScanSubscriber()
    rclpy.spin(laser_scan_subscriber)
    laser_scan_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
