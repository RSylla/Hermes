
import rclpy
from rclpy.node import Node
from serial import Serial
import socket
from pynmeagps import NMEAReader
from time import time

from hermes_interfaces.msg import GpsFixed


class Gps_publisher_node(Node):

    def __init__(self):
        super().__init__('Gps_publisher_node')
        self.publisher_ = self.create_publisher(GpsFixed, 'gps_data', 10)
        timer_period = 0.05  # seconds
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
        if time() - self.start_time > 2:
            self.send_diff_corrections(self.corr_host, self.corr_port, self.stream)
            self.time = time()
        
        msg = GpsFixed()    

        gps_data = self.recieve_gps_data(self.stream)
        data = NMEAReader.parse(gps_data)
        if data.msgID == "UBX":   
             if data.msgId == "00":
                print(data)
                msg.diff_age = float(data.diffAge) if not data.diffAge == "" else float(99.99)
                msg.is_corrected = True if msg.diff_age < 5 else False
                msg.message_id = f"{data.msgID} {data.msgId}"
                msg.utc_time = str(data.time)
                msg.latitude = float(data.lat)
                msg.longtitude = float(data.lon)
                msg.north_south = str(data.NS)
                msg.east_west = str(data.EW)
                msg.nav_status = str(data.navStat)
                msg.hor_accuracy = float(data.hAcc)
                msg.ver_accuracy = float(data.vAcc)
                msg.speed_over_ground_kmh = float(data.SOG)
                msg.course_over_ground_deg = float(data.COG)
                msg.vertical_vel_ms = float(data.vVel)
                msg.num_sat = int(data.numSVs)
                
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
