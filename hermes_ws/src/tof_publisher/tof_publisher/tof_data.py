import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

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
        ranges_str = ', '.join([f"{range:.2f}" for range in msg.ranges])
        self.get_logger().info(f'Ranges: [{ranges_str}]')

def main(args=None):
    rclpy.init(args=args)
    laser_scan_subscriber = LaserScanSubscriber()
    rclpy.spin(laser_scan_subscriber)
    laser_scan_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()