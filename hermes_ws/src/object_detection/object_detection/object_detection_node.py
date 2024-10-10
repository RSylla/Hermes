import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
import gi
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
        # Define the GStreamer pipeline using appsrc
        pipeline_str = (
            "appsrc name=source is-live=true block=true format=3 ! "
            "image/jpeg,width=1280,height=720,framerate=30/1 ! "
            "jpegdec ! videoconvert ! "
            "x264enc tune=zerolatency bitrate=1500 speed-preset=superfast ! "
            "rtph264pay name=pay0 pt=96"
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

class YOLOAndLaneDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_and_lane_detection_node')

        # Initialize YOLO model (e.g., yolov8n.pt)
        self.model = YOLO('src/object_detection/object_detection/yolov11n.pt')  # You can change the model here.

        # Publisher for detected objects and lane detection status
        self.publisher_objects_ = self.create_publisher(String, 'detected_objects_and_lanes', 10)
        self.get_logger().info('YOLO and Lane Detection Node has started.')

        # Subscriber to the camera image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Subscribing to the camera image topic
            self.image_callback,
            10)
        self.bridge = CvBridge()  # Used for converting ROS images to OpenCV format
        self.subscription  # Prevents unused variable warning

        # Store the latest frame for the RTSP server
        self.latest_frame = None

        # Start RTSP server
        self.start_rtsp_server()

    def start_rtsp_server(self):
        # Initialize GStreamer
        Gst.init(None)

        # Create RTSP server
        self.server = GstRtspServer.RTSPServer()
        self.server.set_address("10.0.3.225")  # External IP address
        self.server.set_service("8554")  # Set the RTSP port to 8554 (standard port for RTSP)

        # Use custom factory
        self.factory = MyFactory(self)
        self.factory.set_shared(True)

        # Attach the factory to the RTSP server
        self.mount_points = self.server.get_mount_points()
        self.mount_points.add_factory("/stream", self.factory)
        self.server.attach(None)

        self.get_logger().info('RTSP server started, streaming at rtsp://10.0.3.225:8554/stream')

        # Start the GStreamer loop in a separate thread
        GObject.threads_init()
        self.loop = GObject.MainLoop()
        self.create_thread(self.loop.run)

    def create_thread(self, target):
        # Helper function to start the GStreamer loop in a new thread
        import threading
        thread = threading.Thread(target=target, daemon=True)
        thread.start()

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Run YOLO model on the image
            results = self.model(cv_image)
            obj_names = results[0].names

            # Process YOLO results and extract object information
            detected_objects = []
            for result in results:
                for detection in result.boxes:
                    class_id = int(detection.cls)
                    confidence = detection.conf.item()
                    bbox = detection.xyxy.cpu().numpy()  # Bounding box coordinates
                    detected_objects.append(
                        f"Class: {class_id}, Object: {obj_names[class_id]}, Confidence: {confidence:.2f}, BBox: {bbox}"
                    )

            # Perform lane detection using edge detection (Canny) and Hough Lines
            lane_frame = self.detect_lanes(cv_image)

            # Display the frame with YOLO detections (annotated)
            annotated_frame = results[0].plot()  # Annotate the frame with YOLO detection results

            # Overlay lane detection result on the annotated frame
            combined_frame = cv2.addWeighted(annotated_frame, 0.8, lane_frame, 1, 1)

            # Convert OpenCV image to GStreamer buffer
            success, buffer = cv2.imencode(".jpg", combined_frame)
            if success:
                self.latest_frame = buffer.tobytes()

            # Publish the detected objects as a string message
            detected_objects_str = '\n'.join(detected_objects)
            self.publisher_objects_.publish(String(data=detected_objects_str))

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

    def detect_lanes(self, frame):
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # Apply Canny edge detector
        edges = cv2.Canny(blur, 50, 150)

        # Define region of interest (mask)
        height, width = frame.shape[:2]
        mask = np.zeros_like(edges)
        polygon = np.array([[
            (int(width * 0.1), height),
            (int(width * 0.9), height),
            (int(width * 0.55), int(height * 0.6)),
            (int(width * 0.45), int(height * 0.6))
        ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)

        # Apply mask to edges
        masked_edges = cv2.bitwise_and(edges, mask)

        # Perform Hough Line Transformation to detect lanes
        lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, threshold=100, minLineLength=100, maxLineGap=50)
        lane_image = np.zeros_like(frame)

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(lane_image, (x1, y1), (x2, y2), (0, 255, 0), 5)

        return lane_image

def main(args=None):
    rclpy.init(args=args)
    node = YOLOAndLaneDetectionNode()

    # Spin to process incoming messages (instead of a manual loop)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import CompressedImage, Image
# from std_msgs.msg import String
# from cv_bridge import CvBridge
# import cv2
# from ultralytics import YOLO
# import numpy as np

# class YOLOAndLaneDetectionNode(Node):
#     def __init__(self):
#         super().__init__('yolo_and_lane_detection_node')

#         # Initialize YOLO model (e.g., yolov8n.pt)
#         self.model = YOLO('src/object_detection/object_detection/yolov11n.pt')  # You can change the model here.

#         # Publisher for detected objects and lane detection status
#         self.publisher_objects_ = self.create_publisher(String, 'detected_objects_and_lanes', 10)
#         self.publisher_image_ = self.create_publisher(CompressedImage, '/camera/image_with_objects', 10)
#         self.get_logger().info('YOLO and Lane Detection Node has started.')

#         # Subscriber to the camera image topic
#         self.subscription = self.create_subscription(
#             Image,
#             '/camera/image_raw',  # Subscribing to the camera image topic
#             self.image_callback,
#             10)
#         self.bridge = CvBridge()  # Used for converting ROS images to OpenCV format
#         self.subscription  # Prevents unused variable warning

#     def image_callback(self, msg):
#         try:
#             # Convert ROS Image message to OpenCV image
#             cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#             # Run YOLO model on the image
#             results = self.model(cv_image)
#             obj_names = results[0].names

#             # Process YOLO results and extract object information
#             detected_objects = []
#             for result in results:
#                 for detection in result.boxes:                                       
#                     class_id = int(detection.cls)
#                     confidence = detection.conf.item()
#                     bbox = detection.xyxy.cpu().numpy()  # Bounding box coordinates
#                     detected_objects.append(
#                         f"Class: {class_id}, Object: {obj_names[class_id]}, Confidence: {confidence:.2f}, BBox: {bbox}"
#                     )

#             # Perform lane detection using edge detection (Canny) and Hough Lines
#             lane_frame = self.detect_lanes(cv_image)

#             # Display the frame with YOLO detections (annotated)
#             annotated_frame = results[0].plot()  # Annotate the frame with YOLO detection results

#             # Overlay lane detection result on the annotated frame
#             combined_frame = cv2.addWeighted(annotated_frame, 0.8, lane_frame, 1, 1)

#             # Encode the combined frame as a JPEG to publish as a CompressedImage
#             compressed_image_msg = CompressedImage()
#             compressed_image_msg.header.stamp = self.get_clock().now().to_msg()
#             compressed_image_msg.format = "jpeg"
#             compressed_image_msg.data = np.array(cv2.imencode('.jpg', combined_frame)[1]).tobytes()

#             # Publish the compressed image to the /camera/image_with_objects/compressed topic
#             self.publisher_image_.publish(compressed_image_msg)

#             # Publish the detected objects as a string message
#             detected_objects_str = '\n'.join(detected_objects)
#             self.publisher_objects_.publish(String(data=detected_objects_str))

#         except Exception as e:
#             self.get_logger().error(f"Error processing image: {str(e)}")

#     def detect_lanes(self, frame):
#         # Convert to grayscale
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#         # Apply Gaussian blur
#         blur = cv2.GaussianBlur(gray, (5, 5), 0)

#         # Apply Canny edge detector
#         edges = cv2.Canny(blur, 50, 150)

#         # Define region of interest (mask)
#         height, width = frame.shape[:2]
#         mask = np.zeros_like(edges)
#         polygon = np.array([[
#             (int(width * 0.1), height),
#             (int(width * 0.9), height),
#             (int(width * 0.55), int(height * 0.6)),
#             (int(width * 0.45), int(height * 0.6))
#         ]], np.int32)
#         cv2.fillPoly(mask, polygon, 255)

#         # Apply mask to edges
#         masked_edges = cv2.bitwise_and(edges, mask)

#         # Perform Hough Line Transformation to detect lanes
#         lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, threshold=100, minLineLength=100, maxLineGap=50)
#         lane_image = np.zeros_like(frame)

#         if lines is not None:
#             for line in lines:
#                 x1, y1, x2, y2 = line[0]
#                 cv2.line(lane_image, (x1, y1), (x2, y2), (0, 255, 0), 5)

#         return lane_image

# def main(args=None):
#     rclpy.init(args=args)
#     node = YOLOAndLaneDetectionNode()

#     # Spin to process incoming messages (instead of a manual loop)
#     rclpy.spin(node)

#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
