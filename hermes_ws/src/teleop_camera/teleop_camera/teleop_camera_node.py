# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import CompressedImage
# import cv2
# import numpy as np
# import gi

# # GStreamer RTSP server
# gi.require_version('Gst', '1.0')
# gi.require_version('GstRtspServer', '1.0')
# from gi.repository import Gst, GstRtspServer, GObject

# class MyFactory(GstRtspServer.RTSPMediaFactory):
#     def __init__(self, node):
#         super(MyFactory, self).__init__()
#         self.node = node

#         # Connect the 'media-configure' signal to the method 'on_media_configure'
#         self.connect("media-configure", self.on_media_configure)

#     def do_create_element(self, url):
#         # Define the GStreamer pipeline using appsrc
#         pipeline_str = (
#             "appsrc name=source is-live=true block=true format=3 ! "
#             "image/jpeg,width=1280,height=720,framerate=30/1 ! "
#             "jpegdec ! videoconvert ! "
#             "x264enc tune=zerolatency bitrate=1500 speed-preset=superfast ! "
#             "rtph264pay name=pay0 pt=96"
#         )
#         return Gst.parse_launch(pipeline_str)

#     def on_media_configure(self, factory, rtsp_media):
#         # Directly get the pipeline from the media object
#         pipeline = rtsp_media.get_element()

#         # Access the appsrc element by name from the pipeline
#         appsrc = pipeline.get_by_name("source")

#         # Connect the "need-data" signal from appsrc to push new data
#         if appsrc:
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
#             CompressedImage,
#             '/camera/image_with_objects',
#             self.image_callback,
#             10)
        
#         # Store the latest frame for GStreamer RTSP server
#         self.latest_frame = None
#         self.hosting_ip = "10.0.3.225"
        
#         # Initialize GStreamer RTSP Server
#         Gst.init(None)
#         self.server = GstRtspServer.RTSPServer()

#         # Set the RTSP server address to external IP and the port to 5000
#         self.server.set_address(self.hosting_ip)  # External IP address
#         self.server.set_service("5000")  # Set the RTSP port to 5000

#         # Use custom factory
#         self.factory = MyFactory(self)
#         self.factory.set_shared(True)

#         # Attach the factory to the RTSP server
#         self.mount_points = self.server.get_mount_points()
#         self.mount_points.add_factory("/stream", self.factory)
#         self.server.attach(None)

#         self.get_logger().info(f"RTSP server is running at rtsp://{self.hosting_ip}:5000/stream")

#         # Start the GStreamer loop in a separate thread
#         GObject.threads_init()
#         self.loop = GObject.MainLoop()
#         self.loop_thread = self.create_thread(self.loop.run)

#     def image_callback(self, msg):
#         # Convert ROS CompressedImage message to an OpenCV image
#         np_arr = np.frombuffer(msg.data, np.uint8)
#         cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

#         # Convert OpenCV image to GStreamer buffer
#         success, buffer = cv2.imencode(".jpg", cv_image)
#         if success:
#             self.latest_frame = buffer.tobytes()

#     def create_thread(self, target):
#         # Helper function to start the GStreamer loop in a new thread
#         import threading
#         thread = threading.Thread(target=target, daemon=True)
#         thread.start()
#         return thread

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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import gi

# GStreamer RTSP server
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GObject

class MyFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self, node):
        super(MyFactory, self).__init__()
        self.node = node

        # Connect the 'media-configure' signal to the method 'on_media_configure'
        self.connect("media-configure", self.on_media_configure)

    def do_create_element(self, url):
        # Define the GStreamer pipeline using appsrc and RTP/UDP streaming
        pipeline_str = (
            "appsrc name=source is-live=true block=true format=3 ! "
            "image/jpeg,width=1280,height=720,framerate=30/1 ! "
            "jpegdec ! videoconvert ! "
            "x264enc tune=zerolatency bitrate=1500 speed-preset=superfast ! "
            "rtph264pay config-interval=1 name=pay0 pt=96"
        )
        return Gst.parse_launch(pipeline_str)

    def on_media_configure(self, factory, rtsp_media):
        # Directly get the pipeline from the media object
        pipeline = rtsp_media.get_element()

        # Access the appsrc element by name from the pipeline
        appsrc = pipeline.get_by_name("source")

        # Connect the "need-data" signal from appsrc to push new data
        if appsrc:
            self.appsrc = appsrc
            self.appsrc.connect("need-data", self.need_data)

    def need_data(self, src, length):
        if self.node.latest_frame is not None:
            buffer = Gst.Buffer.new_wrapped(self.node.latest_frame)
            src.emit("push-buffer", buffer)

class CameraStream(Node):
    def __init__(self):
        super().__init__('camera_stream_node')
        
        # Create a subscription to the CompressedImage topic
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_with_objects',
            self.image_callback,
            10)
        
        # Store the latest frame for GStreamer RTSP server
        self.latest_frame = None
        self.hosting_ip = "10.0.3.225"
        
        # Initialize GStreamer RTSP Server
        Gst.init(None)
        self.server = GstRtspServer.RTSPServer()

        # Set the RTSP server address to external IP and the port to 8554
        self.server.set_address(self.hosting_ip)  # External IP address
        self.server.set_service("8554")  # Set the RTSP port to 8554 (standard port for RTSP)

        # Use custom factory
        self.factory = MyFactory(self)
        self.factory.set_shared(True)

        # Attach the factory to the RTSP server
        self.mount_points = self.server.get_mount_points()
        self.mount_points.add_factory("/stream", self.factory)
        self.server.attach(None)

        self.get_logger().info(f"RTSP server is running at rtsp://{self.hosting_ip}:8554/stream")

        # Start the GStreamer loop in a separate thread
        GObject.threads_init()
        self.loop = GObject.MainLoop()
        self.loop_thread = self.create_thread(self.loop.run)

    def image_callback(self, msg):
        # Convert ROS CompressedImage message to an OpenCV image
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Convert OpenCV image to GStreamer buffer
        success, buffer = cv2.imencode(".jpg", cv_image)
        if success:
            self.latest_frame = buffer.tobytes()

    def create_thread(self, target):
        # Helper function to start the GStreamer loop in a new thread
        import threading
        thread = threading.Thread(target=target, daemon=True)
        thread.start()
        return thread

def main(args=None):
    rclpy.init(args=args)

    node = CameraStream()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down RTSP server.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

