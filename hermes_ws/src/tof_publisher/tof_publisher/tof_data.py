import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class LaserScanSubscriber(Node):

    def __init__(self):
        super().__init__('laser_scan_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Bool, '/tof_data', 10)

        self.do_action_for_sector_1 = False
        self.do_action_for_sector_2 = False
        self.do_action_for_sector_3 = False
        self.do_action_for_sector_4 = False

    def listener_callback(self, msg):
        range_start = 0
        range_end = 227
        sector_size = (range_end - range_start + 1) // 4 

        for sector_num in range(4):
            sector_start = range_start + sector_num * sector_size
            sector_end = sector_start + sector_size - 1

            sector_values = msg.ranges[sector_start:sector_end + 1]
            for range_value in sector_values:  
                if 0.0 < range_value < 1.0 and not math.isnan(range_value):

                    # Actions based on sector
                    if sector_num == 0:  # Sector 1
                        print("Sector 1 danger: Value =", range_value)  
                        self.do_action_for_sector_1 = True 
                    elif sector_num == 1:  # Sector 2
                        print("Sector 2 danger: Value =", range_value)
                        self.do_action_for_sector_2 = True  
                    elif sector_num == 2:  # Sector 3
                        print("Sector 3 danger: Value =", range_value)
                        self.do_action_for_sector_3 = True
                    elif sector_num == 3:  # Sector 4
                        print("Sector 4 danger: Value =", range_value)
                        self.do_action_for_sector_4 = True

        # Publish the actions
        self.publish_actions()

    def publish_actions(self):
        msg = Bool()
        msg.data = self.do_action_for_sector_1 or self.do_action_for_sector_2 or self.do_action_for_sector_3 or self.do_action_for_sector_4
        print("Publishing:", msg.data)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    laser_scan_subscriber = LaserScanSubscriber()
    rclpy.spin(laser_scan_subscriber)
    laser_scan_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
