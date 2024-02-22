import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math 

class LidarToTofProcessor(Node):
    def __init__(self):
        super().__init__('lidar_to_tof_processor')

        # Subscriber to lidar data
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # Publisher for processed tof data
        self.tof_pub = self.create_publisher(LaserScan, '/tof_data', 10) 

    def listener_callback(self, msg):
        numerical_ranges = [r for r in msg.ranges if isinstance(r, (int, float)) and not math.isnan(r)]

        if numerical_ranges:
            self.get_logger().info('Numerical ranges: %s' % numerical_ranges)
        else:
            self.get_logger().info('No numerical, non-NaN ranges found.')


def main(args=None):
    rclpy.init(args=args)
    lidar_to_tof_processor = LidarToTofProcessor()
    rclpy.spin(lidar_to_tof_processor)
    lidar_to_tof_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()