import rclpy
from rclpy.node import Node
from serial import Serial
import socket
from pynmeagps import NMEAReader
from time import time

from std_msgs.msg import String
# from msg import GpsFixed


class Gps_publisher_node(Node):

    def __init__(self):
        super().__init__('Gps_publisher_node')
        self.publisher_ = self.create_publisher(String, 'gps_data', 10)
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
        print("corrections sent!")        

    def recieve_gps_data(self, stream):
        gps_data = stream.readline()
        return gps_data

    def timer_callback(self):
        if time() - self.start_time > 1.5:
            self.send_diff_corrections(self.corr_host, self.corr_port, self.stream)
            self.time = time()
        
        msg = String()

        gps_data = self.recieve_gps_data(self.stream)
        nmea_data = NMEAReader.parse(gps_data)
        if nmea_data.msgID == "UBX":   
             if nmea_data.msgId == "00":           
                msg.data = str([nmea_data.hAcc, nmea_data.vAcc])
                
        self.publisher_.publish(msg)
                


def main(args=None):
    rclpy.init(args=args)

    gps_node= Gps_publisher_node()

    rclpy.spin(gps_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
