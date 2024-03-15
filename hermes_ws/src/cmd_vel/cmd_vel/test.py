import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class SimplePublisher(Node):

    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.subscription = self.create_subscription(
            Bool,
            '/tof_data',
            self.lidar_callback,
            10)
        self.get_logger().info('SimplePublisher node initialized')

    def lidar_callback(self, msg):
        self.get_logger().info(f'Received Bool message: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()

    try:
        rclpy.spin(simple_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        simple_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
