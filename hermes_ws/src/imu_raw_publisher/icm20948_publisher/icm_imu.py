import math
import qwiic_icm20948
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from rclpy.clock import Clock

class ICM20948Node(Node):
    def __init__(self):
        super().__init__("icm20948_node")
        
        # Logger
        self.get_logger().info("Initializing ICM20948 Node")
        
        # Parameters
        self.declare_and_get_parameters()
        
        # IMU instance
        self.initialize_imu()
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, "/imu/data_raw", 50)
        self.mag_pub = self.create_publisher(MagneticField, "/imu/mag", 50)
        
        # Timer for publishing
        self.create_timer(1.0 / self.pub_rate, self.publish_data)
        
        # Diagnostics
        self.last_pub_time = self.get_clock().now()
        self.pub_count = 0

    def declare_and_get_parameters(self):
        self.declare_parameter("i2c_address", 0x69)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("pub_rate", 50)
        
        self.i2c_addr = self.get_parameter("i2c_address").value
        self.frame_id = self.get_parameter("frame_id").value
        self.pub_rate = self.get_parameter("pub_rate").value
        
        self.get_logger().info(f"I2C Address: {self.i2c_addr}")
        self.get_logger().info(f"Frame ID: {self.frame_id}")
        self.get_logger().info(f"Publishing rate: {self.pub_rate} Hz")

    def initialize_imu(self):
        try:
            self.imu = qwiic_icm20948.QwiicIcm20948(address=self.i2c_addr)
            if not self.imu.begin():
                raise RuntimeError("IMU initialization failed")
            self.imu.setFullScaleRangeGyro(qwiic_icm20948.dps2000)
            self.imu.setFullScaleRangeAccel(qwiic_icm20948.gpm16)
            self.get_logger().info("IMU initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize IMU: {str(e)}")
            rclpy.shutdown()

    def publish_data(self):
        try:
            self.imu.getAgmt()  # Always get the latest data
            
            imu_msg = Imu()
            mag_msg = MagneticField()
            
            current_time = self.get_clock().now()
            imu_msg.header.stamp = current_time.to_msg()
            imu_msg.header.frame_id = self.frame_id
            
            # Set IMU data
            imu_msg.linear_acceleration.x = self.imu.axRaw * 9.81 / 2048.0
            imu_msg.linear_acceleration.y = self.imu.ayRaw * 9.81 / 2048.0
            imu_msg.linear_acceleration.z = self.imu.azRaw * 9.81 / 2048.0
            
            imu_msg.angular_velocity.x = self.imu.gxRaw * math.pi / (16.4 * 180)
            imu_msg.angular_velocity.y = self.imu.gyRaw * math.pi / (16.4 * 180)
            imu_msg.angular_velocity.z = self.imu.gzRaw * math.pi / (16.4 * 180)
            
            imu_msg.orientation_covariance[0] = -1
            
            # Set magnetometer data
            mag_msg.header.stamp = imu_msg.header.stamp
            mag_msg.header.frame_id = self.frame_id
            mag_msg.magnetic_field.x = self.imu.mxRaw * 1e-6 / 0.15
            mag_msg.magnetic_field.y = self.imu.myRaw * 1e-6 / 0.15
            mag_msg.magnetic_field.z = self.imu.mzRaw * 1e-6 / 0.15
            
            # Publish messages
            self.imu_pub.publish(imu_msg)
            self.mag_pub.publish(mag_msg)
            
            # Update diagnostics
            self.pub_count += 1
            if (current_time - self.last_pub_time).nanoseconds >= 1e9:
                actual_rate = self.pub_count / ((current_time - self.last_pub_time).nanoseconds / 1e9)
                self.get_logger().info(f"Actual publishing rate: {actual_rate:.2f} Hz")
                self.pub_count = 0
                self.last_pub_time = current_time
                
        except Exception as e:
            self.get_logger().error(f"Error in publish_data: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    try:
        icm20948_node = ICM20948Node()
        rclpy.spin(icm20948_node)
    except Exception as e:
        print(f"Unhandled exception: {str(e)}")
    finally:
        if 'icm20948_node' in locals():
            icm20948_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
