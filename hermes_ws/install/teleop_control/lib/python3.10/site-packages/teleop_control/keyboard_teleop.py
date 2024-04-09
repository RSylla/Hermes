#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSPresetProfiles
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import serial
from serial.tools import list_ports
import time

def get_serial_port():
    # List all available serial ports
    available_ports = list(list_ports.comports())
    # Try to find /dev/ttyACM0, and if not found, use /dev/ttyACM1
    for port in available_ports:
        if '/dev/ttyACM2' in port.device:
            return '/dev/ttyACM2'
    for port in available_ports:
        if '/dev/ttyACM1' in port.device:
            return '/dev/ttyACM1'
    for port in available_ports:
        if '/dev/ttyACM0' in port.device:
            return '/dev/ttyACM0'
    return None  # Return None if neither port is found

serial_port = get_serial_port()
if serial_port is None:
    print("No suitable serial port found.")
    sys.exit(1)

# Initialize arduino_serial with the correct port
arduino_serial = serial.Serial(serial_port, 115200, timeout=1)

settings = termios.tcgetattr(sys.stdin)

msg = """
Control Your Robot!
---------------------------
Use keyboard keys to control the robot:

   w
a  s  d

w/x : increase/decrease linear speed
e/c : increase/decrease angular speed

CTRL-C to quit
"""

keyBindings = {
    'w': 'forward',
    's': 'backward',
    'a': 'turn_left',
    'd': 'turn_right',
    'x': 'stop',
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    rclpy.init()
    node = rclpy.create_node('teleop_twist_keyboard')
    pub = node.create_publisher(Twist, 'cmd_vel', QoSPresetProfiles.SYSTEM_DEFAULT.value)

    last_time_sent = time.time()
    rate = 20  # Frequency in Hz
    period = 1.0 / rate  # Period in seconds

    try:
        print(msg)
        while True:
            key = getKey()
            current_time = time.time()

            # Check if enough time has passed since the last command was sent
            if current_time - last_time_sent >= period:
                if key in keyBindings:
                    command = keyBindings[key]
                    arduino_serial.write(command.encode() + b'\n')
                    last_time_sent = current_time  # Update the time last command was sent

                elif key == '\x03':  # CTRL-C
                    break

    except Exception as e:
        print(e)

    finally:
        arduino_serial.write(b'stop\n')  # Ensure robot stops when script ends
        arduino_serial.close()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()