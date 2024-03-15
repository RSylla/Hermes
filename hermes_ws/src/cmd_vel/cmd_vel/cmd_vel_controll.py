import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import serial
import struct
from serial import SerialException
from rclpy.qos import QoSProfile

class SimplePublisher(Node):

    def __init__(self):
        super().__init__('cmd_vel_publisher')
        qos_profile = QoSProfile(depth=10)
        print(f"QoS Profile: {qos_profile}")
        self.subscription = self.create_subscription(
            Bool,
            '/tof_data',
            self.lidar_callback,
            qos_profile=qos_profile)

        #self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription  # Not needed if not used later

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
                self.get_logger().error(f"Failed to connect on {port}")  # Use error logging
        raise SerialException("No available serial ports found")

    def lidar_callback(self, msg):
        if msg.data:
            self.motorspeedflag = 1
        else:
            self.motorspeedflag = 0

        # Send speed command over serial:
        data_to_send = struct.pack('ff', self.motorspeedflag, 0.0)  # Assuming only linear speed
        try:
            self.serial_port.write(data_to_send)
        except SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    arduino_bridge = SimplePublisher()

    try:
        rclpy.spin(arduino_bridge)
    except KeyboardInterrupt:  # Allow graceful shutdown with Ctrl-C
        pass
    finally:
        arduino_bridge.serial_port.close()  
        arduino_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
