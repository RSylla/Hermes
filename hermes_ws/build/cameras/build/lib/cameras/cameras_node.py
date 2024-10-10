import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RealSenseCameraNode(Node):
    def __init__(self):
        super().__init__('realsense_camera_node')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        # Configure RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

        # Start RealSense streaming
        self.pipeline.start(self.config)

        self.timer = self.create_timer(0.03, self.publish_frame)  # 30 FPS (0.03 seconds per frame)

    def publish_frame(self):
        # Wait for a coherent pair of frames
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        # Convert images to numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Convert OpenCV image to ROS2 Image message
        image_message = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")

        # Publish the image message
        self.publisher_.publish(image_message)
        self.get_logger().info('Published an image frame.')

    def stop_camera(self):
        self.pipeline.stop()

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseCameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_camera()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
