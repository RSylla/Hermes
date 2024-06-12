#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from hermes_interfaces.msg import GpsFixed

class GpsConverterNode(Node):

    def __init__(self):
        super().__init__('gps_converter_node')
        self.subscription = self.create_subscription(
            GpsFixed,
            'gps_data',
            self.gps_callback,
            10)
        self.publisher_ = self.create_publisher(NavSatFix, 'gps/fix', 10)

    def gps_callback(self, msg: GpsFixed):
        navsatfix_msg = NavSatFix()
        navsatfix_msg.header.stamp = self.get_clock().now().to_msg()
        navsatfix_msg.header.frame_id = 'gps'
        navsatfix_msg.status.status = NavSatStatus.STATUS_FIX if msg.is_corrected else NavSatStatus.STATUS_NO_FIX
        navsatfix_msg.status.service = NavSatStatus.SERVICE_GPS
        navsatfix_msg.latitude = msg.latitude
        navsatfix_msg.longitude = msg.longtitude
        navsatfix_msg.altitude = 0.0  # Set this to the appropriate altitude if available
        navsatfix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.publisher_.publish(navsatfix_msg)
        self.get_logger().info(f"Published NavSatFix data: {navsatfix_msg.latitude}, {navsatfix_msg.longitude}, {navsatfix_msg.altitude}")

def main(args=None):
    rclpy.init(args=args)
    gps_converter_node = GpsConverterNode()
    try:
        rclpy.spin(gps_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        gps_converter_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

