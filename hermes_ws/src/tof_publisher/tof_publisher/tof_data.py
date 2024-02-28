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
        numerical_value_positions = [(i, range) for i, range in enumerate(msg.ranges) if isinstance(range, (int, float)) and not math.isnan(range)]
        self.get_logger().info(f'Numerical values and their positions in ranges: {numerical_value_positions}')

def main(args=None):
    rclpy.init(args=args)
    laser_scan_subscriber = LaserScanSubscriber()
    rclpy.spin(laser_scan_subscriber)
    laser_scan_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
