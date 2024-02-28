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

        # Initialize flags and timers
        self.sector_1_flag = 0
        self.sector_2_flag = 0
        self.sector_3_flag = 0
        self.sector_4_flag = 0

        self.sector_1_timer = self.create_timer(1.0, self.reset_sector_1_flag)  # 1-second timer
        self.sector_2_timer = self.create_timer(1.0, self.reset_sector_2_flag)
        self.sector_3_timer = self.create_timer(1.0, self.reset_sector_3_flag)
        self.sector_4_timer = self.create_timer(1.0, self.reset_sector_4_flag)

    def do_action_for_sector_1(self):
        # Implement actions for sector 1 warning here
        pass

    def do_action_for_sector_2(self):
        # Implement actions for sector 2 warning here
        pass

    def do_action_for_sector_3(self):
        # Implement actions for sector 3 warning here
        pass

    def do_action_for_sector_4(self):
        # Implement actions for sector 4 warning here
        pass

    def reset_sector_1_flag(self):
        self.sector_1_flag = 0

    def reset_sector_2_flag(self):
        self.sector_2_flag = 0

    def reset_sector_3_flag(self):
        self.sector_3_flag = 0

    def reset_sector_4_flag(self):
        self.sector_4_flag = 0

    def listener_callback(self, msg):
        range_start = 0
        range_end = 227
        sector_size = (range_end - range_start + 1) // 4 

        for sector_num in range(4):
            sector_start = range_start + sector_num * sector_size
            sector_end = sector_start + sector_size - 1

            sector_values = msg.ranges[sector_start:sector_end + 1]
            for range_value in sector_values:  
                if range_value < 1.0 and not math.isnan(range_value):

                    # Actions based on sector
                    if sector_num == 0:  # Sector 1
                        self.sector_1_flag += 1
                        if self.sector_1_flag == 2:
                            print("Sector 1 danger")
                            self.do_action_for_sector_1()  
                    elif sector_num == 1:  # Sector 2
                        self.sector_2_flag += 1
                        if self.sector_2_flag == 2:
                            print("Sector 2 danger")
                            self.do_action_for_sector_2()  
                    elif sector_num == 2:  # Sector 3
                        self.sector_3_flag += 1
                        if self.sector_3_flag == 2:
                            print("Sector 3 danger")
                            self.do_action_for_sector_3()
                    elif sector_num == 3:  # Sector 4
                        self.sector_4_flag += 1
                        if self.sector_4_flag == 2:
                            print("Sector 4 danger")
                            self.do_action_for_sector_4() 

def main(args=None):
    rclpy.init(args=args)
    laser_scan_subscriber = LaserScanSubscriber()
    rclpy.spin(laser_scan_subscriber)
    laser_scan_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    print("Activating LaserScanSubscriber node...")
    main()
