#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import qwiic_icm20948
import time
import sys

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.imu = qwiic_icm20948.QwiicIcm20948()
        self.timer = self.create_timer(0.1, self.timer_callback)  # Adjust the timer period as needed.

        if not self.imu.connected:
            self.get_logger().error("The Qwiic ICM20948 device isn't connected to the system. Please check your connection")
            sys.exit(1)  # Exit if the device is not connected.

        self.imu.begin()

    def timer_callback(self):
        if self.imu.dataReady():
            self.imu.getAgmt()  # Updates all sensor values
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"

            # Gyroscope data (convert to radians per second)
            imu_msg.angular_velocity.x = self.imu.gxRaw / 131.0  # Assuming the gyro range is +/- 250 degrees/sec
            imu_msg.angular_velocity.y = self.imu.gyRaw / 131.0
            imu_msg.angular_velocity.z = self.imu.gzRaw / 131.0

            # Accelerometer data (convert to meters per second squared)
            imu_msg.linear_acceleration.x = self.imu.axRaw * 9.81 / 16384.0  # Assuming the accel range is +/- 2g
            imu_msg.linear_acceleration.y = self.imu.ayRaw * 9.81 / 16384.0
            imu_msg.linear_acceleration.z = self.imu.azRaw * 9.81 / 16384.0

            self.publisher_.publish(imu_msg)
        else:
            self.get_logger().info("Waiting for data")

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = IMUPublisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

