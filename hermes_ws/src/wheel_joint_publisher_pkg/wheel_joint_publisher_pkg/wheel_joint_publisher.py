# wheel_joint_publisher.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32

class WheelJointPublisher(Node):
    def __init__(self):
        super().__init__('wheel_joint_publisher')

        self.left_ticks_sub = self.create_subscription(
            Int32,
            'left_wheel_ticks',
            self.left_ticks_callback,
            50)
        self.right_ticks_sub = self.create_subscription(
            Int32,
            'right_wheel_ticks',
            self.right_ticks_callback,
            50)

        self.joint_state_pub = self.create_publisher(
            JointState,
            'joint_states',
            50)

        self.left_tick_count = 0
        self.right_tick_count = 0
        self.wheel_radius = 0.260
        self.ticks_per_revolution = 90

        self.timer = self.create_timer(0.02, self.publish_joint_states)

    def left_ticks_callback(self, msg):
        self.left_tick_count = msg.data

    def right_ticks_callback(self, msg):
        self.right_tick_count = msg.data

    def publish_joint_states(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['base_front_left_wheel_joint', 'base_front_right_wheel_joint']

        # Convert tick counts to wheel rotations in radians
        left_position = (self.left_tick_count / self.ticks_per_revolution) * 2 * 3.14159265359
        right_position = (self.right_tick_count / self.ticks_per_revolution) * 2 * 3.14159265359

        joint_state.position = [left_position, right_position]

        self.joint_state_pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    wheel_joint_publisher = WheelJointPublisher()
    rclpy.spin(wheel_joint_publisher)
    wheel_joint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

