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
        num_numerical_values = sum(1 for range in msg.ranges if isinstance(range, (int, float)))
        self.get_logger().info(f'Number of numerical values in ranges: {num_numerical_values}')



def main(args=None):
    rclpy.init(args=args)
    laser_scan_subscriber = LaserScanSubscriber()
    rclpy.spin(laser_scan_subscriber)
    laser_scan_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
