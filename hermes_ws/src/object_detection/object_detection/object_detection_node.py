import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
# import torch
import cv2
from ultralytics import YOLO

class ObjectDetection(Node):

    def __init__(self):
        super().__init__('objects_detection')

        # Initialize publishers
        self.objects_publisher = self.create_publisher(String, 'object_detection', 10)
        self.image_teleop_publisher = self.create_publisher(Image, 'image_teleop', 10)

        # Subscribe to color and depth image topics
        self.color_subscription = self.create_subscription(
            Image,
            'camera/image_raw',  # Subscribing to the raw image topic
            self.image_callback, 10)
        
        self.depth_subscription = self.create_subscription(
            Image,
            'camera/image_depth',  # Subscribing to the depth image topic
            self.depth_callback, 10)
        
        # Load YOLOv5 model with CUDA enabled
        # self.model = torch.hub.load('/home/hermes/yolov5', 
        #                             'custom', 
        #                             source="local",
        #                             path="/home/hermes/Hermes/hermes_ws/src/cameras/cameras/yolov5m_Objects365.pt", 
        #                             force_reload=True).cuda().eval()
        # self.path_to_model = "/home/hermes/Hermes/hermes_ws/src/object_detection/object_detection/yolov5m_Objects365.pt"
        self.model = YOLO("yolov11n.pt").to("cuda")

        # Only process detections for required classes
        # self.required_classes = {"Person", "Car", "Bus", "Bicycle", "Motorcycle", "Traffic light", "Stop sign", 
        #                          "Ambulance", "Firetruck", "Police car", "Taxi", "Truck", "Van", "Helmet", 
        #                          "Traffic cone", "Street sign", "Barrier", "Crosswalk", "Pedestrian", "Road", 
        #                          "Bridge"}

        # Initialize CvBridge for image conversion
        self.bridge = CvBridge()
        self.color_image = None  # Placeholder for color image
        self.depth_image = None  # Placeholder for depth image
        self.detections = []  # Store detections to use for image drawing

        # Timer to run object detection at 10 Hz (10 times per second)
        self.detection_timer = self.create_timer(1.0 / 3.0, self.perform_detection)

        # Timer to publish image at 30 Hz (30 times per second)
        self.image_publish_timer = self.create_timer(1.0 / 15.0, self.publish_image_with_bounding_boxes)

    def depth_callback(self, msg):
        # Store depth image for later use, converting it directly
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def image_callback(self, msg):
        # Store the color image for processing
        self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def perform_detection(self):
        if self.color_image is None:
            self.get_logger().info("No color image received yet for detection.")
            return
        
        # Perform YOLOv5 object detection
        # with torch.no_grad():
        #     results = self.model(self.color_image)
        results = self.model.predict(self.color_image)

        # Prepare to collect detections
        detections = []
        depth_image_available = self.depth_image is not None

        for *box, conf, cls in results.xyxy[0].cpu().numpy():
            class_name = self.model.names[int(cls)]
            if class_name in self.required_classes:
                x1, y1, x2, y2 = map(int, box)
                confidence = conf

                # If depth image is available, calculate distance
                distance = -1
                if depth_image_available:
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    if 0 <= center_x < self.depth_image.shape[1] and 0 <= center_y < self.depth_image.shape[0]:
                        distance = self.depth_image[center_y, center_x] * 0.001  # Convert from mm to meters

                # Append the result
                detections.append((class_name, confidence, distance, (x1, y1, x2, y2)))

        # Sort detections by nearest distance first (ignoring invalid distances)
        detections.sort(key=lambda x: x[2] if x[2] != -1 else float('inf'))

        # Store the detections for use in image drawing
        self.detections = detections

        # Publish detections as a simple formatted string
        if detections:
            objects_msg = String()
            objects_msg.data = ', '.join(
                [f"{cls_name} {conf:.2f} {dist:.2f} [{x1},{y1},{x2},{y2}]"
                 for cls_name, conf, dist, (x1, y1, x2, y2) in detections]
            )
            self.objects_publisher.publish(objects_msg)

    def publish_image_with_bounding_boxes(self):
        if self.color_image is None:
            self.get_logger().info("No color image received yet for image publishing.")
            return

        # Copy the image to avoid modifying the original
        modified_image = self.color_image.copy()

        # Draw bounding boxes and labels if there are detections
        for class_name, confidence, distance, (x1, y1, x2, y2) in self.detections:
            label = f"{class_name} {confidence:.2f} {distance:.2f}m"
            box_color = (0, 255, 0)  # Green bounding box
            font_color = (255, 255, 255)  # White text
            line_thickness = 2
            font_scale = 0.6
            font_thickness = 1

            # Draw the bounding box
            cv2.rectangle(modified_image, (x1, y1), (x2, y2), box_color, line_thickness)

            # Put the label above the bounding box
            (text_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)
            cv2.rectangle(modified_image, (x1, y1 - text_height - 10), (x1 + text_width, y1), box_color, -1)  # Background for text
            cv2.putText(modified_image, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_color, font_thickness)

        # Convert the modified image with bounding boxes back to ROS Image message
        modified_image_msg = self.bridge.cv2_to_imgmsg(modified_image, encoding='bgr8')

        # Publish the image with bounding boxes to 'image_teleop'
        self.image_teleop_publisher.publish(modified_image_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetection()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import CompressedImage
# from std_msgs.msg import String
# from cv_bridge import CvBridge
# import numpy as np
# import torch
# import cv2

# class ObjectDetection(Node):

#     def __init__(self):
#         super().__init__('objects_detection')

#         # Initialize publishers
#         self.objects_publisher = self.create_publisher(String, 'object_detection', 10)
#         self.image_teleop_publisher = self.create_publisher(CompressedImage, 'image_teleop/compressed', 10)

#         # Subscribe to the compressed image topic
#         self.color_subscription = self.create_subscription(
#             CompressedImage,
#             'camera/image_raw/compressed',  # Subscribing to the compressed image topic
#             self.image_callback, 10)
        
#         # Depth image subscription (if required for distance calculations)
#         self.depth_subscription = self.create_subscription(
#             CompressedImage,
#             'camera/image_depth/compressed',  # Subscribing to the depth image topic
#             self.depth_callback, 10)
        
#         # Load YOLOv5 model with CUDA enabled
#         self.model = torch.hub.load('/home/hermes/yolov5', 
#                                     'custom', 
#                                     source="local",
#                                     path="/home/hermes/Hermes/hermes_ws/src/cameras/cameras/yolov5m_Objects365.pt", 
#                                     force_reload=True).cuda().eval()

#         # Only process detections for required classes
#         self.required_classes = {"Person", "Car", "Bus", "Bicycle", "Motorcycle", "Traffic light", "Stop sign", 
#                                  "Ambulance", "Firetruck", "Police car", "Taxi", "Truck", "Van", "Helmet", 
#                                  "Traffic cone", "Street sign", "Barrier", "Crosswalk", "Pedestrian", "Road", 
#                                  "Bridge"}

#         # Initialize CvBridge for image conversion
#         self.bridge = CvBridge()
#         self.color_image = None  # Placeholder for color image
#         self.depth_image = None  # Placeholder for depth image
#         self.detections = []  # Store detections to use for image drawing

#         # Timer to run object detection at 10 Hz (10 times per second)
#         self.detection_timer = self.create_timer(1.0 / 10.0, self.perform_detection)

#         # Timer to publish image at 30 Hz (30 times per second)
#         self.image_publish_timer = self.create_timer(1.0 / 30.0, self.publish_image_with_bounding_boxes)

#     def depth_callback(self, msg):
#         # Convert compressed depth image back to OpenCV format
#         np_arr = np.frombuffer(msg.data, np.uint8)
#         self.depth_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)

#     def image_callback(self, msg):
#         # Decompress the image and store it
#         np_arr = np.frombuffer(msg.data, np.uint8)
#         self.color_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

#     def perform_detection(self):
#         if self.color_image is None:
#             self.get_logger().info("No color image received yet for detection.")
#             return
        
#         # Perform YOLOv5 object detection
#         with torch.no_grad():
#             results = self.model(self.color_image)

#         # Prepare to collect detections
#         detections = []
#         depth_image_available = self.depth_image is not None

#         for *box, conf, cls in results.xyxy[0].cpu().numpy():
#             class_name = self.model.names[int(cls)]
#             if class_name in self.required_classes:
#                 x1, y1, x2, y2 = map(int, box)
#                 confidence = conf

#                 # If depth image is available, calculate distance
#                 distance = -1
#                 if depth_image_available:
#                     center_x = (x1 + x2) // 2
#                     center_y = (y1 + y2) // 2
#                     if 0 <= center_x < self.depth_image.shape[1] and 0 <= center_y < self.depth_image.shape[0]:
#                         distance = self.depth_image[center_y, center_x] * 0.001  # Convert from mm to meters

#                 # Append the result
#                 detections.append((class_name, confidence, distance, (x1, y1, x2, y2)))

#         # Sort detections by nearest distance first (ignoring invalid distances)
#         detections.sort(key=lambda x: x[2] if x[2] != -1 else float('inf'))

#         # Store the detections for use in image drawing
#         self.detections = detections

#         # Publish detections as a simple formatted string
#         if detections:
#             objects_msg = String()
#             objects_msg.data = ', '.join(
#                 [f"{cls_name} {conf:.2f} {dist:.2f} [{x1},{y1},{x2},{y2}]"
#                  for cls_name, conf, dist, (x1, y1, x2, y2) in detections]
#             )
#             self.objects_publisher.publish(objects_msg)

#     def publish_image_with_bounding_boxes(self):
#         if self.color_image is None:
#             self.get_logger().info("No color image received yet for image publishing.")
#             return

#         # Copy the image to avoid modifying the original
#         modified_image = self.color_image.copy()

#         # Draw bounding boxes and labels if there are detections
#         for class_name, confidence, distance, (x1, y1, x2, y2) in self.detections:
#             label = f"{class_name} {confidence:.2f} {distance:.2f}m"
#             box_color = (0, 255, 0)  # Green bounding box
#             font_color = (255, 255, 255)  # White text
#             line_thickness = 2
#             font_scale = 0.6
#             font_thickness = 1

#             # Draw the bounding box
#             cv2.rectangle(modified_image, (x1, y1), (x2, y2), box_color, line_thickness)

#             # Put the label above the bounding box
#             (text_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)
#             cv2.rectangle(modified_image, (x1, y1 - text_height - 10), (x1 + text_width, y1), box_color, -1)  # Background for text
#             cv2.putText(modified_image, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_color, font_thickness)

#         # Compress the modified image into JPEG format
#         success, compressed_image = cv2.imencode('.jpg', modified_image)
#         if success:
#             # Convert the compressed image to ROS CompressedImage message
#             compressed_image_msg = CompressedImage()
#             compressed_image_msg.header.stamp = self.get_clock().now().to_msg()
#             compressed_image_msg.format = "jpeg"
#             compressed_image_msg.data = np.array(compressed_image).tobytes()

#             # Publish the compressed image with bounding boxes to 'image_teleop/compressed'
#             self.image_teleop_publisher.publish(compressed_image_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = ObjectDetection()
    
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Shutting down.")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
