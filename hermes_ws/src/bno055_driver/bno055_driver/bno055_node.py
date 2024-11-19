#!/usr/bin/env python3
"""
BNO055 IMU ROS2 Publisher optimized for Navigation 2 (Nav2)
This node publishes IMU data and TF2 transforms for use with Nav2.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Temperature
from geometry_msgs.msg import TransformStamped
import tf2_ros
import time
import struct
import smbus2
from threading import Lock
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class BNO055Node(Node):
    # I2C addresses and constants
    DEFAULT_ADDRESS = 0x28
    REG_CHIP_ID = 0x00
    REG_OPR_MODE = 0x3D
    REG_PWR_MODE = 0x3E
    REG_SYS_TRIGGER = 0x3F
    REG_TEMP = 0x34
    REG_CALIB_STAT = 0x35
    OPERATION_MODE_NDOF = 0x0C
    REG_QUATERNION_DATA = 0x20
    REG_GYRO_DATA = 0x14
    REG_LINEAR_ACCEL_DATA = 0x28

    def __init__(self):
        super().__init__('bno055_node')

        # Initialize parameters with Nav2-specific defaults
        self.declare_parameters(
            namespace='',
            parameters=[
                ('frame_id', 'imu_link'),
                ('parent_frame', 'base_link'),  # Parent frame for TF
                ('bus_num', 1),
                ('i2c_address', self.DEFAULT_ADDRESS),
                ('update_rate', 100.0),  # Increased for better navigation performance
                ('publish_tf', True),
            ]
        )

        # Get parameters
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.bus_num = self.get_parameter('bus_num').get_parameter_value().integer_value
        self.address = self.get_parameter('i2c_address').get_parameter_value().integer_value
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value

        # Set up I2C
        self.bus = smbus2.SMBus(self.bus_num)
        self.bus_lock = Lock()

        # Configure QoS for Nav2
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # Initialize publishers with Nav2-optimized QoS
        self.imu_pub = self.create_publisher(Imu, 'imu', qos_profile)
        self.temp_pub = self.create_publisher(Temperature, 'imu/temp', qos_profile)

        # Initialize TF2 broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Create timer
        self.timer = self.create_timer(1.0 / self.update_rate, self.publish_imu_data)

        # Initialize sensor
        try:
            self._initialize_sensor()
            # Wait for initial calibration
            self._wait_for_calibration()
            self.get_logger().info('BNO055 node initialized and calibrated.')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize BNO055 sensor: {e}')
            self.destroy_node()

    def _wait_for_calibration(self):
        """Wait for initial calibration with timeout."""
        timeout = 30  # 30 seconds timeout
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            sys, gyro, accel, mag = self.get_calibration_status()
            if sys >= 2 and gyro >= 2 and accel >= 2:
                self.get_logger().info('Initial calibration achieved')
                return True
            time.sleep(0.1)
        
        self.get_logger().warning('Calibration timeout - proceeding with partial calibration')
        return False

    def get_calibration_status(self):
        """Get the calibration status of the sensor."""
        calib_stat = self._read_byte(self.REG_CALIB_STAT)
        sys = (calib_stat >> 6) & 0x03
        gyro = (calib_stat >> 4) & 0x03
        accel = (calib_stat >> 2) & 0x03
        mag = calib_stat & 0x03
        return sys, gyro, accel, mag

    def _initialize_sensor(self):
        """Initialize the BNO055 sensor with Nav2-optimized settings."""
        try:
            # Verify chip ID
            chip_id = self._read_byte(self.REG_CHIP_ID)
            if chip_id != 0xA0:
                raise RuntimeError(f"Invalid chip ID: 0x{chip_id:02X}, expected 0xA0")

            # Reset the device
            self._write_byte(self.REG_SYS_TRIGGER, 0x20)
            time.sleep(0.7)  # Wait for reset

            # Wait for chip ID to become available
            timeout = time.time() + 1.0  # 1-second timeout
            while time.time() < timeout:
                try:
                    chip_id = self._read_byte(self.REG_CHIP_ID)
                    if chip_id == 0xA0:
                        break
                except:
                    time.sleep(0.01)
            else:
                raise RuntimeError("Timeout waiting for chip ID")

            # Set power mode to normal
            self._write_byte(self.REG_PWR_MODE, 0x00)
            time.sleep(0.1)  # Wait for power mode change

            # Set operation mode to NDOF
            self.set_mode(self.OPERATION_MODE_NDOF)
            time.sleep(1.0)  # Give extra time for mode switch

            self.get_logger().info('Sensor initialized successfully')

        except Exception as e:
            self.get_logger().error(f'Error in sensor initialization: {str(e)}')
            raise

    def set_mode(self, mode):
        """Set the operation mode of the sensor."""
        try:
            # Switch to config mode
            self._write_byte(self.REG_OPR_MODE, 0x00)  # CONFIG mode
            time.sleep(0.05)  # Wait for mode switch

            # Switch to desired mode
            self._write_byte(self.REG_OPR_MODE, mode)
            time.sleep(0.05)  # Wait for mode switch

            self.get_logger().info(f'Sensor mode set to: {mode}')
        except Exception as e:
            self.get_logger().error(f'Error setting mode: {str(e)}')
            raise

    def publish_imu_data(self):
        """Publish IMU data optimized for Nav2."""
        try:
            current_time = self.get_clock().now()
            
            # Get sensor data
            w, x, y, z = self.get_quaternion()
            gx, gy, gz = self.get_gyro()
            ax, ay, az = self.get_linear_acceleration()

            # Create IMU message with Nav2-optimized covariance
            imu_msg = Imu()
            imu_msg.header.stamp = current_time.to_msg()
            imu_msg.header.frame_id = self.frame_id

            # Set orientation
            imu_msg.orientation.w = w
            imu_msg.orientation.x = x
            imu_msg.orientation.y = y
            imu_msg.orientation.z = z

            # Set angular velocity
            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz

            # Set linear acceleration
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az

            # Set covariance matrices for Nav2
            orientation_cov = [0.001, 0.0, 0.0,
                               0.0, 0.001, 0.0,
                               0.0, 0.0, 0.001]
            angular_vel_cov = [0.0001, 0.0, 0.0,
                               0.0, 0.0001, 0.0,
                               0.0, 0.0, 0.0001]
            linear_accel_cov = [0.0001, 0.0, 0.0,
                                0.0, 0.0001, 0.0,
                                0.0, 0.0, 0.0001]

            imu_msg.orientation_covariance = orientation_cov
            imu_msg.angular_velocity_covariance = angular_vel_cov
            imu_msg.linear_acceleration_covariance = linear_accel_cov

            # Publish IMU data
            self.imu_pub.publish(imu_msg)

            # Publish TF2 transform if enabled
            if self.publish_tf:
                transform = TransformStamped()
                transform.header.stamp = current_time.to_msg()
                transform.header.frame_id = self.parent_frame
                transform.child_frame_id = self.frame_id
                
                # Set the transform orientation from IMU
                transform.transform.rotation.w = w
                transform.transform.rotation.x = x
                transform.transform.rotation.y = y
                transform.transform.rotation.z = z

                # IMU does not provide positional data
                transform.transform.translation.x = 0.0
                transform.transform.translation.y = 0.0
                transform.transform.translation.z = 0.0
                
                self.tf_broadcaster.sendTransform(transform)

        except Exception as e:
            self.get_logger().error(f'Error publishing IMU data: {str(e)}')

    def _read_byte(self, register):
        """Read a single byte from the specified register."""
        with self.bus_lock:
            return self.bus.read_byte_data(self.address, register)

    def _write_byte(self, register, value):
        """Write a single byte to the specified register."""
        with self.bus_lock:
            self.bus.write_byte_data(self.address, register, value)

    def _read_bytes(self, register, length):
        """Read multiple bytes starting from the specified register."""
        with self.bus_lock:
            return self.bus.read_i2c_block_data(self.address, register, length)

    def get_quaternion(self):
        """Get quaternion data."""
        data = self._read_bytes(self.REG_QUATERNION_DATA, 8)
        w, x, y, z = struct.unpack('<hhhh', bytes(data))
        scale = 1.0 / (1 << 14)
        return w * scale, x * scale, y * scale, z * scale

    def get_gyro(self):
        """Get gyroscope data in rad/s."""
        data = self._read_bytes(self.REG_GYRO_DATA, 6)
        x, y, z = struct.unpack('<hhh', bytes(data))
        scale = 0.001065264436  # Convert to rad/s
        return x * scale, y * scale, z * scale

    def get_linear_acceleration(self):
        """Get linear acceleration in m/s^2."""
        data = self._read_bytes(self.REG_LINEAR_ACCEL_DATA, 6)
        x, y, z = struct.unpack('<hhh', bytes(data))
        scale = 0.01  # 1 LSB = 0.01 m/s^2
        return x * scale, y * scale, z * scale

    def destroy_node(self):
        """Clean shutdown."""
        try:
            self.bus.close()
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BNO055Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
