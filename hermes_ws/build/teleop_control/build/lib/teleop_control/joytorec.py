import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray
import serial
import struct
from serial import SerialException

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        self.subscription = self.create_subscription(
            UInt16MultiArray,
            'gamepad_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Initialize serial communication with Arduino
        self.serial_port = self.initialize_serial(['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2'])

    def initialize_serial(self, port_list, baud_rate= 115200):
        for port in port_list:
            try:
                serial_port = serial.Serial(port, baud_rate)
                serial_port.flushInput()  # Clear the serial buffer
                self.get_logger().info(f"Connected to {port}")
                return serial_port
            except SerialException:
                self.get_logger().info(f"Failed to connect on {port}")
        raise SerialException("No available serial ports found")

    def listener_callback(self, msg):
        uint16_data = msg.data
        self.serial_port.flushInput()  # Clear the serial buffer
        self.serial_port.write(struct.pack('!2H', *uint16_data))
        self.get_logger().info(f'Received and forwarded: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    arduino_bridge = ArduinoBridge()

    try:
        rclpy.spin(arduino_bridge)
    finally:
        arduino_bridge.serial_port.close()  # Close the serial port
        arduino_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()