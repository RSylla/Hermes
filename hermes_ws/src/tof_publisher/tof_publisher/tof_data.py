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
            '/ldlidar_node/scan',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Bool, '/tof_data', 10)
        self.stop = False
        #self.do_action_for_sector_1 = False
        #self.do_action_for_sector_2 = False
        #self.do_action_for_sector_3 = False
        #self.do_action_for_sector_4 = False

    def publish_actions(self):
        msg = Bool()
        msg.data = self.stop #self.do_action_for_sector_1 or self.do_action_for_sector_2 or self.do_action_for_sector_3 or self.do_action_for_sector_4
        self.publisher.publish(msg)
        print("Published message!")

    def listener_callback(self, msg):
        range_start = 0
        range_end = 227
        sector_size = (range_end - range_start + 1) // 4
        self.stop = False  # Reset the stop flag for each new message
    
        for sector_num in range(4):
            sector_start = range_start + sector_num * sector_size
            sector_end = sector_start + sector_size - 1
    
            sector_values = msg.ranges[sector_start:sector_end + 1]
            for range_value in sector_values:
                if 0.0 < range_value < 0.5 and not math.isnan(range_value):
                    self.stop = True  # Set the flag to True if a valid range value is found
                    #print(range_value, self.stop)
                    self.publish_actions()  # Publish actions immediately
                elif range_value > 0.5:
                    self.stop = False  # Reset the flag if a range_value greater than 1.0 is encountered
                    #print(self.stop)
                    self.publish_actions()  # Publish actions immediately
                    break  # Exit the loop as we don't need to check further in this sector
                
            #if self.stop:
               # break  # Exit the loop if the flag is already set
            
                    #if sector_num == 0:  # Sector 1
                    #    self.sector_1_value = range_value
                    #    self.do_action_for_sector_1 = True 
                    #elif sector_num == 1:  # Sector 2
                    #    self.sector_2_value = range_value
                    #    self.do_action_for_sector_2 = True  
                    #elif sector_num == 2:  # Sector 3
                    #    self.sector_3_value = range_value
                    #    self.do_action_for_sector_3 = True
                    #elif sector_num == 3:  # Sector 4
                    #    self.sector_4_value = range_value
                    #    self.do_action_for_sector_4 = True
                    #else:
                    #    self.do_action_for_sector_1 = False
                    #    self.do_action_for_sector_2 = False
                    #    self.do_action_for_sector_3 = False
                    #    self.do_action_for_sector_4 = False
        #print(self.stop)#[self.do_action_for_sector_1, self.sector_1_value],[self.do_action_for_sector_2, self.sector_2_value],[self.do_action_for_sector_3, self.sector_3_value], [self.do_action_for_sector_4, self.sector_4_value])     
                #print(self.do_action_for_sector_1, self.do_action_for_sector_2, self.do_action_for_sector_3, self.do_action_for_sector_4)

        # Publish the actions
        #self.publish_actions()

def main(args=None):
    rclpy.init(args=args)
    laser_scan_subscriber = LaserScanSubscriber()
    rclpy.spin(laser_scan_subscriber)
    laser_scan_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
