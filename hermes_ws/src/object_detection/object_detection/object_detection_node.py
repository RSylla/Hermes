# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import String
# from cv_bridge import CvBridge
# import cv2
# from ultralytics import YOLO

# class YOLOObjectDetectionNode(Node):
#     def __init__(self):
#         super().__init__('yolo_object_detection_node')

#         # Initialize YOLO model (e.g., yolov8n.pt)
#         self.model = YOLO('src/object_detection/object_detection/yolov11n.pt')  # You can change the model here.

#         # Publisher for detected objects
#         self.publisher_ = self.create_publisher(String, 'detected_objects', 10)
#         self.get_logger().info('YOLO Object Detection Node has started.')

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

#             # Process the results and extract object information
#             detected_objects = []
#             for result in results:
#                 for detection in result.boxes:                                       
#                     class_id = int(detection.cls)
#                     confidence = detection.conf.item()
#                     bbox = detection.xyxy.cpu().numpy()  # Bounding box coordinates
#                     detected_objects.append(
#                         f"Class: {class_id}, Object: {obj_names[class_id]}, Confidence: {confidence:.2f}, BBox: {bbox}"
#                     )

#             # Publish the detected objects as a string message
#             detected_objects_str = '\n'.join(detected_objects)
#             self.publisher_.publish(String(data=detected_objects_str))

#             # Display the frame with detections (optional, for debugging purposes)
#             annotated_frame = results[0].plot()  # Annotate the frame with detection results
#             cv2.imshow('YOLO Detections', annotated_frame)

#             # Exit on 'q' key press
#             if cv2.waitKey(1) & 0xFF == ord('q'):
#                 self.get_logger().info('Shutting down...')
#                 cv2.destroyAllWindows()
#                 rclpy.shutdown()

#         except Exception as e:
#             self.get_logger().error(f"Error processing image: {str(e)}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = YOLOObjectDetectionNode()

#     # Spin to process incoming messages (instead of a manual loop)
#     rclpy.spin(node)

#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import String
# from cv_bridge import CvBridge
# import cv2
# from ultralytics import YOLO
# import subprocess

# class YOLOObjectDetectionNode(Node):
#     def __init__(self):
#         super().__init__('yolo_object_detection_node')

#         # Initialize YOLO model (e.g., yolov8n.pt)
#         self.model = YOLO('src/object_detection/object_detection/yolov11n.pt')  # You can change the model here.

#         # Publisher for detected objects
#         self.publisher_ = self.create_publisher(String, 'detected_objects', 10)
#         self.get_logger().info('YOLO Object Detection Node has started.')

#         # Subscriber to the camera image topic
#         self.subscription = self.create_subscription(
#             Image,
#             '/camera/image_raw',  # Subscribing to the camera image topic
#             self.image_callback,
#             10)
#         self.bridge = CvBridge()  # Used for converting ROS images to OpenCV format
#         self.subscription  # Prevents unused variable warning

#         # GStreamer pipeline setup for SSH streaming
#         self.gstreamer_process = None
#         self.gstreamer_pipeline = (
#             "appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=ultrafast ! h264parse ! "
#             "mpegtsmux ! hlssink max-files=5 target-duration=5 location=/var/www/html/segment%05d.ts "
#             "playlist-location=/var/www/html/playlist.m3u8"
#         )

#     def image_callback(self, msg):
#         try:
#             # Convert ROS Image message to OpenCV image
#             cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#             # Run YOLO model on the image
#             results = self.model(cv_image)
#             obj_names = results[0].names

#             # Process the results and extract object information
#             detected_objects = []
#             for result in results:
#                 for detection in result.boxes:                                       
#                     class_id = int(detection.cls)
#                     confidence = detection.conf.item()
#                     bbox = detection.xyxy.cpu().numpy()  # Bounding box coordinates
#                     detected_objects.append(
#                         f"Class: {class_id}, Object: {obj_names[class_id]}, Confidence: {confidence:.2f}, BBox: {bbox}"
#                     )

#             # Publish the detected objects as a string message
#             detected_objects_str = '\n'.join(detected_objects)
#             self.publisher_.publish(String(data=detected_objects_str))

#             # Send the video frame through GStreamer
#             if self.gstreamer_process is None:
#                 self.gstreamer_process = subprocess.Popen(self.gstreamer_pipeline, shell=True, stdin=subprocess.PIPE)

#             # Write frame to GStreamer pipeline
#             self.gstreamer_process.stdin.write(cv_image.tobytes())

#             # Exit on 'q' key press
#             if cv2.waitKey(1) & 0xFF == ord('q'):
#                 self.get_logger().info('Shutting down...')
#                 if self.gstreamer_process is not None:
#                     self.gstreamer_process.terminate()
#                 rclpy.shutdown()

#         except Exception as e:
#             self.get_logger().error(f"Error processing image: {str(e)}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = YOLOObjectDetectionNode()

#     # Spin to process incoming messages (instead of a manual loop)
#     rclpy.spin(node)

#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import subprocess
import numpy as np

class YOLOObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_object_detection_node')

        # Initialize YOLO model (e.g., yolov8n.pt)
        self.model = YOLO('src/object_detection/object_detection/yolov11n.pt')  # You can change the model here.

        # Publisher for detected objects
        self.publisher_ = self.create_publisher(String, 'detected_objects', 10)
        self.get_logger().info('YOLO Object Detection Node has started.')

        # Subscriber to the camera image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Subscribing to the camera image topic
            self.image_callback,
            10)
        self.bridge = CvBridge()  # Used for converting ROS images to OpenCV format

        # GStreamer pipeline setup for streaming
        self.gstreamer_process = None
        self.gstreamer_pipeline = (
            "appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=ultrafast ! h264parse ! "
            "mpegtsmux ! hlssink max-files=5 target-duration=5 location=/var/www/html/segment%05d.ts "
            "playlist-location=/var/www/html/playlist.m3u8"
        )

        # Start GStreamer pipeline in constructor
        try:
            self.gstreamer_process = subprocess.Popen(self.gstreamer_pipeline, shell=True, stdin=subprocess.PIPE)
        except Exception as e:
            self.get_logger().error(f"Failed to start GStreamer pipeline: {str(e)}")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Run YOLO model on the image
            results = self.model(cv_image)
            obj_names = results[0].names

            # Process the results and extract object information
            detected_objects = []
            for result in results:
                for detection in result.boxes:
                    class_id = int(detection.cls)
                    confidence = detection.conf.item()
                    bbox = detection.xyxy.cpu().numpy()  # Bounding box coordinates
                    detected_objects.append(
                        f"Class: {class_id}, Object: {obj_names[class_id]}, Confidence: {confidence:.2f}, BBox: {bbox}"
                    )

            # Publish the detected objects as a string message
            detected_objects_str = '\n'.join(detected_objects)
            self.publisher_.publish(String(data=detected_objects_str))

            # Check if GStreamer process is still running before writing frames
            if self.gstreamer_process is not None and self.gstreamer_process.poll() is None:
                # Convert OpenCV image (BGR format) to RGB, then encode as raw bytes for GStreamer
                rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                gstreamer_frame = np.array(rgb_image).tobytes()
                self.gstreamer_process.stdin.write(gstreamer_frame)
            else:
                self.get_logger().error("GStreamer pipeline is not active.")

            # Display the frame with detections (optional, for debugging purposes)
            annotated_frame = results[0].plot()  # Annotate the frame with detection results
            cv2.imshow('YOLO Detections', annotated_frame)

            # Exit on 'q' key press or window close
            if cv2.waitKey(1) & 0xFF == ord('q') or cv2.getWindowProperty('YOLO Detections', cv2.WND_PROP_VISIBLE) < 1:
                self.get_logger().info('Shutting down...')
                cv2.destroyAllWindows()
                self.destroy_node()
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

    def destroy_node(self):
        # Ensure GStreamer process is cleaned up
        if self.gstreamer_process is not None:
            self.get_logger().info("Terminating GStreamer pipeline.")
            self.gstreamer_process.terminate()
            self.gstreamer_process = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YOLOObjectDetectionNode()

    # Spin to process incoming messages (instead of a manual loop)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
