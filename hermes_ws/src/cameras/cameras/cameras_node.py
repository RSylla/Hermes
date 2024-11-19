import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import pyrealsense2 as rs
import numpy as np
from cv_bridge import CvBridge  # To convert OpenCV images to ROS Image messages

class RealSense(Node):
    def __init__(self):
        super().__init__('realsense_camera')

        # Initialize the RealSense pipeline and configure streams
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # 30 FPS, 640x480 resolution
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # Start RealSense pipeline
        self.pipeline.start(config)

        # ROS publishers for color and depth images
        self.color_image_publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.depth_image_publisher = self.create_publisher(Image, '/camera/image_depth', 10)

        # Initialize CvBridge for converting images
        self.bridge = CvBridge()

        # Use a timer to periodically fetch and publish frames
        self.timer = self.create_timer(1 / 20.0, self.process_frames)  # 30 Hz

        self.get_logger().info("RealSense D400 camera feed is running.")

    def process_frames(self):
        try:
            # Wait for frames from the RealSense pipeline
            frames = self.pipeline.wait_for_frames()
            
            # Get color and depth frames
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            # Ensure both color and depth frames are valid before processing
            if not color_frame or not depth_frame:
                return

            # Convert RealSense frames directly to ROS Image messages without extra conversions
            color_image_msg = self.bridge.cv2_to_imgmsg(np.asarray(color_frame.get_data()), encoding="bgr8")
            depth_image_msg = self.bridge.cv2_to_imgmsg(np.asarray(depth_frame.get_data()), encoding="passthrough")

            # Publish the images only if they are valid
            self.color_image_publisher.publish(color_image_msg)
            self.depth_image_publisher.publish(depth_image_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing RealSense frames: {e}")

def main(args=None):
    rclpy.init(args=args)
    realsense_node = RealSense()

    try:
        rclpy.spin(realsense_node)
    except KeyboardInterrupt:
        realsense_node.get_logger().info("Shutting down RealSense node.")
    finally:
        # Ensure the RealSense pipeline stops cleanly
        realsense_node.pipeline.stop()
        realsense_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
