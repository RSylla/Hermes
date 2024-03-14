import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import serial
import struct
from serial import SerialException

class SimplePublisher(Node):

    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.subscription = self.create_subscription(
            Bool,  # Specify the message type you're subscribing to
            '/tof_data',
            self.lidar_callback,
            qos_profile=rclpy.qos.qos_profile_sensor_data)
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription  
        self.serial_port = self.initialize_serial(['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2'])
        self.motorspeedflag = 0
    
    def initialize_serial(self, port_list, baud_rate=115200):
        for port in port_list:
            try:
                serial_port = serial.Serial(port, baud_rate)
                serial_port.flushInput()  
                self.get_logger().info(f"Connected to {port}")
                return serial_port
            except SerialException:
                self.get_logger().info(f"Failed to connect on {port}")
        raise SerialException("No available serial ports found")
    
    def lidar_callback(self, msg):
        if msg.data == True:
            self.motorspeedflag = 1
            data_to_send = struct.pack('ff', self.motorspeedflag)
            self.serial_port.write(data_to_send)
        else:
            self.motorspeedflag = 0
            data_to_send = struct.pack('ff', self.motorspeedflag)
            self.serial_port.write(data_to_send)

def main(args=None):
    rclpy.init(args=args)
    arduino_bridge = SimplePublisher()
    try:
        rclpy.spin(arduino_bridge)
    finally:
        arduino_bridge.serial_port.close()  
        arduino_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
