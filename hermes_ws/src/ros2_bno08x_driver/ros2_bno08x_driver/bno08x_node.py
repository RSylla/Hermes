#!/usr/bin/env python3
"""High-performance BNO08X IMU ROS2 Node optimized for Nav2"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import tf2_ros
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from smbus2 import SMBus
from ros2_bno08x_driver.bno08x_i2c import (
    BNO08X_I2C, 
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE, 
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GAME_ROTATION_VECTOR,
    BNO_REPORT_LINEAR_ACCELERATION,
    DEFAULT_REPORT_INTERVAL
)

class BNO08XNode(Node):
    def __init__(self):
        super().__init__('bno08x_node')
        
        # Core parameters - hardcoded for performance
        self.frame_id = 'imu_link'
        self.parent_frame = 'base_link'
        self.i2c_bus_num = 1
        self.address = 0x4A
        self.update_rate = 100.0  # Maximum stable rate
        self.publish_tf = True
        self.report_interval = DEFAULT_REPORT_INTERVAL

        # Pre-allocate messages and initialize sensor
        self._setup_messages()
        self._setup_publishers()
        self._initialize_sensor()
        
        # High-frequency timer
        self.timer = self.create_timer(1.0 / self.update_rate, self.publish_imu_data)

    def _setup_messages(self):
        """Pre-allocate and initialize all message structures"""
        self._imu_msg = Imu()
        self._mag_msg = MagneticField()
        self._transform = tf2_ros.TransformStamped()

        # Pre-set all static fields
        self._imu_msg.header.frame_id = self.frame_id
        self._mag_msg.header.frame_id = self.frame_id

        self._transform.header.frame_id = self.parent_frame
        self._transform.child_frame_id = self.frame_id
        self._transform.transform.translation.x = 0.0
        self._transform.transform.translation.y = 0.0
        self._transform.transform.translation.z = 0.0

        # Pre-set covariance matrices (3x3 as flat arrays)
        self._imu_msg.orientation_covariance = [
            0.0005, 0.0, 0.0,
            0.0, 0.0005, 0.0,
            0.0, 0.0, 0.0005
        ]
        self._imu_msg.angular_velocity_covariance = [
            0.00002, 0.0, 0.0,
            0.0, 0.00002, 0.0,
            0.0, 0.0, 0.00002
        ]
        self._imu_msg.linear_acceleration_covariance = [
            0.00002, 0.0, 0.0,
            0.0, 0.00002, 0.0,
            0.0, 0.0, 0.00002
        ]

    def _setup_publishers(self):
        """Setup ROS publishers with optimized QoS"""
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # Nav2 requires RELIABLE
            history=HistoryPolicy.KEEP_LAST,
            depth=1,  # Minimum buffer
            durability=DurabilityPolicy.VOLATILE
        )

        # Create publishers
        self.imu_pub = self.create_publisher(Imu, 'imu', qos)  # Standard topic
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', qos)
        
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def _initialize_sensor(self):
        """Initialize IMU with optimal settings"""
        try:
            # Initialize I2C bus
            i2c_bus = SMBus(self.i2c_bus_num)
            
            # Create sensor instance
            self.imu = BNO08X_I2C(
                i2c_bus=i2c_bus,
                address=self.address,
                debug=False  # Never enable debug
            )
            
            # Soft reset and wait
            self.imu.soft_reset()
            time.sleep(0.1)
            
            # Enable essential features in optimal order
            self.imu.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
            time.sleep(0.05)
            self.imu.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            time.sleep(0.05)
            self.imu.enable_feature(BNO_REPORT_GYROSCOPE)
            time.sleep(0.05)
            self.imu.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)
            time.sleep(0.05)
            self.imu.enable_feature(BNO_REPORT_MAGNETOMETER)
            time.sleep(0.05)
            
            # Basic calibration
            self.imu.calibration()
            time.sleep(0.1)
            
            # Quick tare
            self.imu.tare()
            time.sleep(0.1)

            # Save calibration data
            try:
                self.imu.save_calibration_data()
            except:
                self.get_logger().warn('Could not save calibration data')

            self.get_logger().info('IMU initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Sensor initialization failed: {str(e)}')
            raise

    def publish_imu_data(self):
        """Fast publish cycle with minimal overhead"""
        try:
            # Get timestamp once
            current_time = self.get_clock().now().to_msg()
            
            # Get game quaternion for stable orientation
            quat = self.imu.game_quaternion
            
            # Update IMU message fields
            self._imu_msg.header.stamp = current_time
            self._imu_msg.orientation.w = quat[3]  # w is last in quaternion
            self._imu_msg.orientation.x = quat[0]
            self._imu_msg.orientation.y = quat[1]
            self._imu_msg.orientation.z = quat[2]

            # Get motion data
            gyro = self.imu.gyro
            accel = self.imu.linear_acceleration  # Using linear acceleration
            
            # Update motion data
            self._imu_msg.angular_velocity.x = gyro[0]
            self._imu_msg.angular_velocity.y = gyro[1]
            self._imu_msg.angular_velocity.z = gyro[2]
            
            self._imu_msg.linear_acceleration.x = accel[0]
            self._imu_msg.linear_acceleration.y = accel[1]
            self._imu_msg.linear_acceleration.z = accel[2]

            # Magnetometer data (if available)
            try:
                mag = self.imu.magnetic
                self._mag_msg.header.stamp = current_time
                self._mag_msg.magnetic_field.x = mag[0]
                self._mag_msg.magnetic_field.y = mag[1]
                self._mag_msg.magnetic_field.z = mag[2]
                self.mag_pub.publish(self._mag_msg)
            except:
                pass  # Skip magnetometer on error

            # Publish IMU data
            self.imu_pub.publish(self._imu_msg)

            # Update transform if enabled
            if self.publish_tf:
                self._transform.header.stamp = current_time
                self._transform.transform.rotation = self._imu_msg.orientation
                self.tf_broadcaster.sendTransform(self._transform)

        except Exception as e:
            self.get_logger().debug(f'Publish cycle error: {str(e)}')

    def destroy_node(self):
        """Clean shutdown"""
        if hasattr(self, 'timer'):
            self.timer.cancel()
        if hasattr(self, 'imu'):
            try:
                self.imu._i2cbus.close()
            except:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = BNO08XNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
    finally:
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()