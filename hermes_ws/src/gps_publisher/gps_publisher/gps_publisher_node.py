import rclpy
from rclpy.node import Node
from serial import Serial
import socket
from pynmeagps import NMEAReader, NMEAMessage
from time import time
import numpy as np
import pymap3d as pm
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header

class GpsPublisherNode(Node):

    def __init__(self):
        super().__init__('Gps_publisher_node')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps/fix', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.stream = Serial(port="/dev/ttyACM0", baudrate=9600, timeout=0.1)
        self.corr_host = "213.168.5.170"
        self.corr_port = 8002
        self.start_time = time()

    def send_diff_corrections(self, host, port, stream):
        corr_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        with corr_socket as s:
            s.connect((host, port))
            corrections = s.recv(1024)
        stream.write(corrections)
        self.get_logger().info("Corrections sent!")        

    def receive_gps_data(self, stream):
        gps_data = stream.readline().decode('ascii', errors='replace').strip()
        self.get_logger().info(f"Received GPS data: {gps_data}")
        return gps_data

    def timer_callback(self):
        if time() - self.start_time > 2.5:
            self.send_diff_corrections(self.corr_host, self.corr_port, self.stream)
            self.start_time = time()

        msg = NavSatFix()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "gps"
        msg.header = header

        gps_data = self.receive_gps_data(self.stream)

        try:
            data = NMEAReader.parse(gps_data)
            self.get_logger().info(f"Parsed GPS data: {data}")

            if isinstance(data, NMEAMessage) and data.msgID == "UBX" and data.msgId == "00":
                msg.status.status = NavSatStatus.STATUS_FIX
                msg.status.service = NavSatStatus.SERVICE_GPS

                # Check if the fields are not empty before converting to float
                if data.lat and data.lon and data.altRef:
                    msg.latitude = float(data.lat)
                    msg.longitude = float(data.lon)
                    msg.altitude = float(data.altRef)

                    x, y, z = pm.geodetic2enu(float(data.lat),
                                              float(data.lon),
                                              float(data.altRef),
                                              58.3428685594,
                                              25.5692475361,
                                              91.357)

                    # Create a covariance matrix and ensure it's 3x3
                    covariance_matrix = np.cov(np.array([x, y, z]))
                    if covariance_matrix.shape == (3, 3):
                        msg.position_covariance = covariance_matrix.flatten().tolist()
                    else:
                        msg.position_covariance = [0.0] * 9  # Default to zero covariance if incorrect shape
                    msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN
                else:
                    msg.status.status = NavSatStatus.STATUS_NO_FIX
                    msg.status.service = 0
                    msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                    msg.position_covariance = [0.0] * 9  # Ensure covariance list has 9 elements
            else:
                msg.status.status = NavSatStatus.STATUS_NO_FIX
                msg.status.service = 0
                msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                msg.position_covariance = [0.0] * 9  # Ensure covariance list has 9 elements

        except Exception as e:
            self.get_logger().error(f"Error parsing GPS data: {e}")
            msg.status.status = NavSatStatus.STATUS_NO_FIX
            msg.status.service = 0
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            msg.position_covariance = [0.0] * 9  # Ensure covariance list has 9 elements

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    gps_node = GpsPublisherNode()

    rclpy.spin(gps_node)

    gps_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

