import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
import re
from cv_bridge import CvBridge

class CameraStream(Node):
    def __init__(self):
        super().__init__('camera_stream_node')

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Create subscriptions for image and object detection topics
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            1)
        
        self.detection_subscription = self.create_subscription(
            String,
            '/object_detection',
            self.detection_callback,
            10)

        # Publisher for the modified image with bounding boxes
        self.image_publisher = self.create_publisher(Image, '/image_teleop', 10)

        # Store the latest detections
        self.latest_detections = []
        self.timer = self.create_timer(1 / 30.0, self.image_callback)  # 30 Hz

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format using CvBridge
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert ROS image to OpenCV: {e}")
            return

        if cv_image is None or cv_image.size == 0:
            self.get_logger().error("Received an invalid image, skipping frame processing.")
            return

        # Log the number of detections to ensure they are being processed
        if not self.latest_detections:
            self.get_logger().info("No detections available to draw.")
        else:
            self.get_logger().info(f"Drawing {len(self.latest_detections)} bounding boxes.")

        # Draw bounding boxes with the latest detections
        self.draw_bounding_boxes(cv_image)

        # Convert the modified OpenCV image back to ROS Image message and publish it
        modified_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.image_publisher.publish(modified_image_msg)

    def detection_callback(self, msg):
        # Parse detection data from the object detection topic
        detection_str = msg.data
        self.get_logger().info(f"Received detections: {detection_str}")

        # Use regex to parse the detection data in the format "class_name confidence distance [x1, y1, x2, y2]"
        detection_pattern = re.compile(r"(\w+) (\d+\.\d+) (\d+\.\d+) \[(\d+), (\d+), (\d+), (\d+)\]")
        self.latest_detections = [
            (match[0], float(match[1]), float(match[2]), (int(match[3]), int(match[4]), int(match[5]), int(match[6])))
            for match in detection_pattern.findall(detection_str)
        ]

    def draw_bounding_boxes(self, image):
        for class_name, confidence, distance, (x1, y1, x2, y2) in self.latest_detections:
            label = f"{class_name} {confidence:.2f} {distance:.2f}m"
            
            # Custom colors and line thickness for bounding box and text
            box_color = (0, 0, 255)  # Red bounding box
            font_color = (255, 255, 255)  # White text
            line_thickness = 3
            font_scale = 0.8
            font_thickness = 2

            # Draw bounding box
            cv2.rectangle(image, (x1, y1), (x2, y2), box_color, line_thickness)

            # Draw label above the bounding box
            (text_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)
            cv2.rectangle(image, (x1, y1 - text_height - 10), (x1 + text_width, y1), box_color, -1)  # Background for text
            cv2.putText(image, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_color, font_thickness)

def main(args=None):
    rclpy.init(args=args)
    node = CameraStream()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down CameraStream node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()





# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import String
# import cv2
# import numpy as np
# import gi
# import threading
# import re
# from cv_bridge import CvBridge

# # GStreamer RTSP server
# gi.require_version('Gst', '1.0')
# gi.require_version('GstRtspServer', '1.0')
# from gi.repository import Gst, GstRtspServer, GObject

# class MyFactory(GstRtspServer.RTSPMediaFactory):
#     def __init__(self, node):
#         super(MyFactory, self).__init__()
#         self.node = node
#         self.set_shared(True)
#         self.connect("media-configure", self.on_media_configure)

#     def do_create_element(self, url):
#         pipeline_str = (
#             "appsrc name=source is-live=true block=true format=3 ! "
#             "image/jpeg,width=1280,height=720,framerate=30/1 ! "
#             "jpegdec ! videoconvert ! "
#             "x264enc tune=zerolatency bitrate=1500 speed-preset=ultrafast ! "
#             "rtph264pay config-interval=1 name=pay0 pt=96"
#         )
#         return Gst.parse_launch(pipeline_str)

#     def on_media_configure(self, factory, rtsp_media):
#         pipeline = rtsp_media.get_element()
#         appsrc = pipeline.get_by_name("source")
#         if appsrc:
#             appsrc.set_property("format", Gst.Format.TIME)
#             self.appsrc = appsrc
#             self.appsrc.connect("need-data", self.need_data)

#     def need_data(self, src, length):
#         if self.node.latest_frame is not None:
#             buffer = Gst.Buffer.new_wrapped(self.node.latest_frame)
#             src.emit("push-buffer", buffer)

# class CameraStream(Node):
#     def __init__(self):
#         super().__init__('camera_stream_node')

#         # Initialize CvBridge
#         self.bridge = CvBridge()

#         # Create subscriptions for image and object detection topics
#         self.image_subscription = self.create_subscription(
#             Image,
#             '/camera/image_raw',
#             self.image_callback,
#             10)
        
#         self.detection_subscription = self.create_subscription(
#             String,
#             '/object_detection',
#             self.detection_callback,
#             10)

#         self.latest_frame = None
#         self.latest_detections = []
#         self.hosting_ip = "10.0.3.225"

#         # Initialize GStreamer RTSP Server
#         Gst.init(None)
#         self.server = GstRtspServer.RTSPServer()
#         self.server.set_address(self.hosting_ip)
#         self.server.set_service("5000")
        
#         # Use custom factory for RTSP stream
#         self.factory = MyFactory(self)
#         self.mount_points = self.server.get_mount_points()
#         self.mount_points.add_factory("/stream", self.factory)
#         self.server.attach(None)

#         self.get_logger().info(f"RTSP server is running at rtsp://{self.hosting_ip}:5000/stream")

#         # Start the GStreamer loop in a separate thread
#         GObject.threads_init()
#         self.loop = GObject.MainLoop()
#         self.loop_thread = threading.Thread(target=self.loop.run, daemon=True)
#         self.loop_thread.start()

#     def image_callback(self, msg):
#         try:
#             # Convert ROS Image to OpenCV format using CvBridge
#             cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         except Exception as e:
#             self.get_logger().error(f"Failed to convert ROS image to OpenCV: {e}")
#             return

#         if cv_image is None or cv_image.size == 0:
#             self.get_logger().error("Received an invalid image, skipping frame processing.")
#             return

#         # Draw bounding boxes with optimized method
#         self.draw_bounding_boxes(cv_image)

#         # Perform encoding and stream in the background
#         threading.Thread(target=self.process_frame, args=(cv_image,), daemon=True).start()

#     def detection_callback(self, msg):
#         detection_str = msg.data
#         detection_pattern = re.compile(r"(\w+) (\d+\.\d+) (\d+\.\d+) \[(\d+), (\d+), (\d+), (\d+)\]")
#         self.latest_detections = [
#             (match[0], float(match[1]), float(match[2]), (int(match[3]), int(match[4]), int(match[5]), int(match[6])))
#             for match in detection_pattern.findall(detection_str)
#         ]

#     def draw_bounding_boxes(self, image):
#         for class_name, confidence, distance, (x1, y1, x2, y2) in self.latest_detections:
#             label = f"{class_name} {confidence:.2f} {distance:.2f}m"
            
#             # Custom colors and line thickness
#             box_color = (0, 0, 255)  # Red bounding box
#             font_color = (255, 255, 255)  # White text
#             line_thickness = 3
#             font_scale = 0.8
#             font_thickness = 2

#             # Draw bounding box
#             cv2.rectangle(image, (x1, y1), (x2, y2), box_color, line_thickness)

#             # Draw label above the bounding box
#             (text_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)
#             cv2.rectangle(image, (x1, y1 - text_height - 10), (x1 + text_width, y1), box_color, -1)  # Background for text
#             cv2.putText(image, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_color, font_thickness)

#     def process_frame(self, frame):
#         success, buffer = cv2.imencode(".jpg", frame)
#         if success:
#             self.latest_frame = buffer.tobytes()

# def main(args=None):
#     rclpy.init(args=args)
#     node = CameraStream()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Shutting down RTSP server.")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()



# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# import cv2
# import numpy as np
# import gi
# import threading

# # GStreamer RTSP server
# gi.require_version('Gst', '1.0')
# gi.require_version('GstRtspServer', '1.0')
# from gi.repository import Gst, GstRtspServer, GObject

# class MyFactory(GstRtspServer.RTSPMediaFactory):
#     def __init__(self, node):
#         super(MyFactory, self).__init__()
#         self.node = node
#         self.set_shared(True)
#         self.connect("media-configure", self.on_media_configure)

#     def do_create_element(self, url):
#         pipeline_str = (
#             "appsrc name=source is-live=true block=true format=3 ! "
#             "image/jpeg,width=1280,height=720,framerate=30/1 ! "
#             "jpegdec ! videoconvert ! "
#             "x264enc tune=zerolatency bitrate=1500 speed-preset=ultrafast ! "
#             "rtph264pay config-interval=1 name=pay0 pt=96"
#         )
#         return Gst.parse_launch(pipeline_str)

#     def on_media_configure(self, factory, rtsp_media):
#         pipeline = rtsp_media.get_element()
#         appsrc = pipeline.get_by_name("source")
#         if appsrc:
#             appsrc.set_property("format", Gst.Format.TIME)
#             self.appsrc = appsrc
#             self.appsrc.connect("need-data", self.need_data)

#     def need_data(self, src, length):
#         if self.node.latest_frame is not None:
#             buffer = Gst.Buffer.new_wrapped(self.node.latest_frame)
#             src.emit("push-buffer", buffer)

# class CameraStream(Node):
#     def __init__(self):
#         super().__init__('camera_stream_node')

#         # Create a subscription to the CompressedImage topic
#         self.subscription = self.create_subscription(
#             Image,
#             '/camera/image_raw',
#             self.image_callback,
#             10)
        
#         # Store the latest frame for GStreamer RTSP server
#         self.latest_frame = None
#         self.hosting_ip = "10.0.3.225"

#         # Initialize GStreamer RTSP Server
#         Gst.init(None)
#         self.server = GstRtspServer.RTSPServer()
#         self.server.set_address(self.hosting_ip)
#         self.server.set_service("5000")

#         # Use custom factory
#         self.factory = MyFactory(self)

#         # Attach the factory to the RTSP server
#         self.mount_points = self.server.get_mount_points()
#         self.mount_points.add_factory("/stream", self.factory)
#         self.server.attach(None)

#         self.get_logger().info(f"RTSP server is running at rtsp://{self.hosting_ip}:5000/stream")

#         # Start the GStreamer loop in a separate thread
#         GObject.threads_init()
#         self.loop = GObject.MainLoop()
#         self.loop_thread = threading.Thread(target=self.loop.run, daemon=True)
#         self.loop_thread.start()

#     def image_callback(self, msg):
#         # Convert ROS CompressedImage message to OpenCV image and encode to JPEG
#         np_arr = np.frombuffer(msg.data, np.uint8)
#         cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

#         # Perform encoding in a background thread to avoid blocking
#         threading.Thread(target=self.process_frame, args=(cv_image,), daemon=True).start()

#     def process_frame(self, frame):
#         success, buffer = cv2.imencode(".jpg", frame)
#         if success:
#             self.latest_frame = buffer.tobytes()

# def main(args=None):
#     rclpy.init(args=args)

#     node = CameraStream()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Shutting down RTSP server.")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# import rclpy
# from rclpy.node import Node  # Make sure Node is imported
# import cv2
# import numpy as np
# import gi
# import threading
# import pyrealsense2 as rs  # Import RealSense library

# # GStreamer RTSP server
# gi.require_version('Gst', '1.0')
# gi.require_version('GstRtspServer', '1.0')
# from gi.repository import Gst, GstRtspServer, GObject

# class MyFactory(GstRtspServer.RTSPMediaFactory):
#     def __init__(self, node):
#         super(MyFactory, self).__init__()
#         self.node = node
#         self.set_shared(True)
#         self.connect("media-configure", self.on_media_configure)

#     def do_create_element(self, url):
#         pipeline_str = (
#             "appsrc name=source is-live=true block=true format=3 ! "
#             "image/jpeg,width=1280,height=720,framerate=30/1 ! "
#             "jpegdec ! videoconvert ! "
#             "x264enc tune=zerolatency bitrate=1500 speed-preset=ultrafast ! "
#             "rtph264pay config-interval=1 name=pay0 pt=96"
#         )
#         return Gst.parse_launch(pipeline_str)

#     def on_media_configure(self, factory, rtsp_media):
#         pipeline = rtsp_media.get_element()
#         appsrc = pipeline.get_by_name("source")
#         if appsrc:
#             appsrc.set_property("format", Gst.Format.TIME)
#             self.appsrc = appsrc
#             self.appsrc.connect("need-data", self.need_data)

#     def need_data(self, src, length):
#         if self.node.latest_frame is not None:
#             buffer = Gst.Buffer.new_wrapped(self.node.latest_frame)
#             src.emit("push-buffer", buffer)

# class CameraStream(Node):
#     def __init__(self):
#         super().__init__('camera_stream_node')

#         # Initialize RealSense pipeline
#         self.pipeline = rs.pipeline()
#         self.config = rs.config()
#         self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

#         # Start the RealSense camera pipeline
#         self.pipeline.start(self.config)
        
#         # Store the latest frame for GStreamer RTSP server
#         self.latest_frame = None
#         self.hosting_ip = "10.0.3.225"

#         # Initialize GStreamer RTSP Server
#         Gst.init(None)
#         self.server = GstRtspServer.RTSPServer()
#         self.server.set_address(self.hosting_ip)
#         self.server.set_service("5000")

#         # Use custom factory
#         self.factory = MyFactory(self)

#         # Attach the factory to the RTSP server
#         self.mount_points = self.server.get_mount_points()
#         self.mount_points.add_factory("/stream", self.factory)
#         self.server.attach(None)

#         self.get_logger().info(f"RTSP server is running at rtsp://{self.hosting_ip}:5000/stream")

#         # Start the GStreamer loop in a separate thread
#         GObject.threads_init()
#         self.loop = GObject.MainLoop()
#         self.loop_thread = threading.Thread(target=self.loop.run, daemon=True)
#         self.loop_thread.start()

#         # Start a thread to continuously grab frames from the RealSense camera
#         self.frame_thread = threading.Thread(target=self.capture_frames, daemon=True)
#         self.frame_thread.start()

#     def capture_frames(self):
#         try:
#             while True:
#                 frames = self.pipeline.wait_for_frames()
#                 color_frame = frames.get_color_frame()

#                 if not color_frame:
#                     continue

#                 # Convert RealSense frame to OpenCV format
#                 frame = np.asanyarray(color_frame.get_data())

#                 # Perform encoding in a background thread to avoid blocking
#                 threading.Thread(target=self.process_frame, args=(frame,), daemon=True).start()

#         except Exception as e:
#             self.get_logger().error(f"Error capturing frames: {e}")
#         finally:
#             self.pipeline.stop()

#     def process_frame(self, frame):
#         success, buffer = cv2.imencode(".jpg", frame)
#         if success:
#             self.latest_frame = buffer.tobytes()

# def main(args=None):
#     rclpy.init(args=args)

#     node = CameraStream()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Shutting down RTSP server.")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
