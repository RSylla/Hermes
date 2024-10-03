import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray

class GamepadPublisher(Node):
    def __init__(self):
        super().__init__('gamepad_publisher')
        self.publisher_ = self.create_publisher(UInt16MultiArray, 'gamepad_data', 10)
        pygame.init()
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.last_joystick_state = {'left_joystick_x': 0, 'left_joystick_y': 0}

    def map_joystick_value(self, value):
        # Map from [-1, 1] to [1000, 2000]
        return int((value + 1) * 500 + 1000)

    def publish_gamepad_data(self):
        pygame.event.pump()
        left_joystick_x = self.map_joystick_value(self.joystick.get_axis(0))
        left_joystick_y = self.map_joystick_value(self.joystick.get_axis(1))

        if (left_joystick_x != self.last_joystick_state['left_joystick_x'] or
            left_joystick_y != self.last_joystick_state['left_joystick_y']):
            
            data = [left_joystick_x, left_joystick_y]
            uint16_data = [int(x) for x in data]
            msg = UInt16MultiArray(data=uint16_data)
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: {msg.data}')
            self.last_joystick_state['left_joystick_x'] = left_joystick_x
            self.last_joystick_state['left_joystick_y'] = left_joystick_y

def main(args=None):
    rclpy.init(args=args)
    gamepad_publisher = GamepadPublisher()

    try:
        while rclpy.ok():
            gamepad_publisher.publish_gamepad_data()
    finally:
        gamepad_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()