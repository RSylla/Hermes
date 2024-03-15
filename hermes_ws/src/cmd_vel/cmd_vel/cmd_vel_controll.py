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
            Bool,
            '/tof_data',
            self.lidar_callback,
            10)  # This line is correct according to the provided code context

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

    def send_motor_command(self, motor_state):
        data_to_send = 't' if motor_state else 'f'
        data_to_send = data_to_send.encode()
        try:
            self.serial_port.write(data_to_send)
        except SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def lidar_callback(self, msg):
        if msg.data == True:
            print("Motorspeedflag: ", self.motorspeedflag)
        else:
            print("Motorspeedflag: ", self.motorspeedflag)
        self.send_motor_command(self.motorspeedflag)

def main(args=None):
    rclpy.init(args=args)
    arduino_bridge = SimplePublisher()

    try:
        rclpy.spin(arduino_bridge)
    except KeyboardInterrupt:  # Allow graceful shutdown with Ctrl-C
        pass
    finally:
        if arduino_bridge.serial_port:
            arduino_bridge.serial_port.close()  # Close the serial port if it exists
        arduino_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()