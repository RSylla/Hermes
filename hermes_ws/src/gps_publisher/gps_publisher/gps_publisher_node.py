import rclpy
from rclpy.node import Node
from serial import Serial
import socket
from pynmeagps import NMEAReader
import time
import numpy as np
import pymap3d as pm
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header
from rclpy.executors import MultiThreadedExecutor
import threading

class CorrectionHandlerNode(Node):

    def __init__(self):
        super().__init__('Correction_handler_node')
        threading.Thread(target=self.run_loop).start()
        self.stream = Serial(port="/dev/ttyACM0", baudrate=115200, timeout=1)
                
        
    def run_loop(self):
        while True:
            host = "213.168.5.170"
            port = 8002
            corr_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)        
            with corr_socket as s:
                s.connect((host, port))
                corrections = s.recv(1024)
            self.stream.write(corrections)
            self.get_logger().info("Corrections sent!")
            time.sleep(2)

class GpsPublisherNode(Node):

    def __init__(self):
        super().__init__('Gps_publisher_node')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps/fix', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.stream = Serial(port="/dev/ttyACM0", baudrate=115200, timeout=1)
        
        # Store last three GPS samples
        self.lat_samples = []
        self.lon_samples = []
        self.alt_samples = []

    def receive_gps_data(self, stream):
        gps_data = stream.readline().decode('ascii', errors='replace').strip()
        return gps_data

    def timer_callback(self):
        msg = NavSatFix()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "gps_node"
        msg.header = header

        gps_data = self.receive_gps_data(self.stream)
        
        if gps_data.startswith("$GNGGA"):
            try:
                data = NMEAReader.parse(gps_data)
                self.get_logger().info(f"Parsed GPS data: lat={data.lat}, lon={data.lon}, alt={data.alt}, quality={data.quality}")
                msg.status.status = NavSatStatus.STATUS_FIX
                msg.status.service = NavSatStatus.SERVICE_GPS                
                
                # Check if the fields are not empty before converting to float
                if data.lat and data.lon and data.alt:                    
                    # Append the new sample to the lists
                    self.lat_samples.append(float(data.lat))
                    self.lon_samples.append(float(data.lon))
                    self.alt_samples.append(float(data.alt))
                    
                    # Keep only the last 3 samples
                    if len(self.lat_samples) > 3:
                        self.lat_samples.pop(0)
                        self.lon_samples.pop(0)
                        self.alt_samples.pop(0)

                    # Only compute the covariance matrix if we have exactly 3 samples
                    if len(self.lat_samples) == 3:
                        # Convert to ENU coordinates
                        ref_lat = self.lat_samples[0]
                        ref_lon = self.lon_samples[0]
                        ref_alt = self.alt_samples[0]
                        
                        x_samples = []
                        y_samples = []
                        z_samples = []

                        for lat, lon, alt in zip(self.lat_samples, self.lon_samples, self.alt_samples):
                            x, y, z = pm.geodetic2enu(lat, lon, alt, ref_lat, ref_lon, ref_alt)
                            x_samples.append(x)
                            y_samples.append(y)
                            z_samples.append(z)

                        # Create an array of ENU samples
                        enu_data = np.array([x_samples, y_samples, z_samples])

                        # Compute the covariance matrix
                        covariance_matrix = np.cov(enu_data)

                        if covariance_matrix.shape == (3, 3):
                            msg.position_covariance = covariance_matrix.flatten().tolist()
                        else:
                            msg.position_covariance = [0.0] * 9  # Default to zero covariance if incorrect shape
                        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN
                    else:
                        # Not enough samples yet
                        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                        msg.position_covariance = [0.0] * 9  # Default covariance
                    
                    msg.latitude = self.lat_samples[-1]
                    msg.longitude = self.lon_samples[-1]
                    msg.altitude = self.alt_samples[-1]                    
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

  try:
    gps_node = GpsPublisherNode()
    corrections_handler = CorrectionHandlerNode()

    executor = MultiThreadedExecutor()
    executor.add_node(gps_node)
    executor.add_node(corrections_handler)

    try:
      executor.spin()
    finally:
      executor.shutdown()
      gps_node.destroy_node()
      corrections_handler.destroy_node()
  finally:
    rclpy.shutdown()

if __name__ == '__main__':
    main()
