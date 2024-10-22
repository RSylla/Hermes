# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import String
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# from ultralytics import YOLO


# class ObjectDetection(Node):
#     def __init__(self):
#         super().__init__('objects_detection')
#         self.bridge = CvBridge()
#         self.color_image = None
#         self.depth_image = None
#         self.detections = []

#         # YOLO model path
#         model_path = "/home/hermes/Hermes/hermes_ws/src/object_detection/object_detection/yolov11n_self_driving_car.pt"
#         self.model = YOLO(model_path).to("cuda")

#         # Publishers and subscribers
#         self.objects_publisher = self.create_publisher(String, 'object_detection', 10)
#         self.image_teleop_publisher = self.create_publisher(Image, 'image_teleop', 10)
#         self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
#         self.create_subscription(Image, 'camera/image_depth', self.depth_callback, 10)
        
#         # Timers for detection and image publishing
#         self.create_timer(1.0 / 3.0, self.perform_detection)  # 3 Hz detection
#         self.create_timer(1.0 / 15.0, self.publish_image_with_bounding_boxes)  # 15 Hz image publishing

#     def depth_callback(self, msg):
#         self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

#     def image_callback(self, msg):
#         self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

#     def perform_detection(self):
#         if self.color_image is None:
#             self.get_logger().info("No color image received yet for detection.")
#             return
        
#         # YOLO object detection
#         results = self.model.predict(self.color_image)
#         self.detections = self.extract_detections(results)
#         self.publish_detections()

#     def extract_detections(self, results):
#         detections = []
#         depth_available = self.depth_image is not None

#         for result in results:
#             for box in result.boxes:
#                 x1, y1, x2, y2 = map(int, box.xyxy[0])
#                 confidence = box.conf[0].item()
#                 class_name = self.model.names[int(box.cls[0].item())]

#                 distance = -1
#                 if depth_available:
#                     center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
#                     if 0 <= center_x < self.depth_image.shape[1] and 0 <= center_y < self.depth_image.shape[0]:
#                         distance = self.depth_image[center_y, center_x] * 0.001  # mm to meters

#                 detections.append((class_name, confidence, distance, (x1, y1, x2, y2)))

#         # Sort detections by nearest distance
#         return sorted(detections, key=lambda x: x[2] if x[2] != -1 else float('inf'))

#     def publish_detections(self):
#         if not self.detections:
#             return

#         objects_msg = String()
#         objects_msg.data = ', '.join(
#             [f"{cls_name} {conf:.2f} {dist:.2f}m [{x1},{y1},{x2},{y2}]"
#              for cls_name, conf, dist, (x1, y1, x2, y2) in self.detections]
#         )
#         self.objects_publisher.publish(objects_msg)

#     def publish_image_with_bounding_boxes(self):
#         if self.color_image is None or not self.detections:
#             return

#         # Draw bounding boxes and labels on the image
#         image = self.color_image.copy()
#         for class_name, confidence, distance, (x1, y1, x2, y2) in self.detections:
#             label = f"{class_name} {confidence:.2f} {distance:.2f}m"
#             cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
#             cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
#         # Convert and publish the image
#         image_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
#         self.image_teleop_publisher.publish(image_msg)


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



import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO


class ObjectDetection(Node):
    def __init__(self):
        super().__init__('objects_detection')
        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.detections = []

        # YOLO model path
        model_path = "/home/hermes/Hermes/hermes_ws/src/object_detection/object_detection/yolov11n_self_driving_car.pt"
        self.model = YOLO(model_path).to("cuda")

        # Publishers and subscribers
        self.objects_publisher = self.create_publisher(String, 'object_detection', 10)
        self.image_teleop_publisher = self.create_publisher(Image, 'image_teleop', 10)
        self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.create_subscription(Image, 'camera/image_depth', self.depth_callback, 10)
        
        # Timers for detection and image publishing
        self.create_timer(1.0 / 3.0, self.perform_detection)  # 3 Hz detection
        self.create_timer(1.0 / 15.0, self.publish_image_with_bounding_boxes)  # 15 Hz image publishing

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def image_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
    def publish_detections(self):
        if not self.detections:
            return

        objects_msg = String()
        objects_msg.data = ', '.join(
            [f"{cls_name} {conf:.2f} {dist:.2f}m [{x1},{y1},{x2},{y2}]"
            for cls_name, conf, dist, (x1, y1, x2, y2) in self.detections]
        )
        self.objects_publisher.publish(objects_msg)

    def perform_detection(self):
        if self.color_image is None:
            self.get_logger().info("No color image received yet for detection.")
            return
        
        # YOLO object detection
        results = self.model.predict(self.color_image)
        self.detections = self.extract_detections(results)
        self.publish_detections()

    def extract_detections(self, results):
        detections = []
        depth_available = self.depth_image is not None

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                confidence = box.conf[0].item()
                class_name = self.model.names[int(box.cls[0].item())]

                distance = -1
                if depth_available:
                    center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
                    if 0 <= center_x < self.depth_image.shape[1] and 0 <= center_y < self.depth_image.shape[0]:
                        distance = self.depth_image[center_y, center_x] * 0.001  # mm to meters

                detections.append((class_name, confidence, distance, (x1, y1, x2, y2)))

        return sorted(detections, key=lambda x: x[2] if x[2] != -1 else float('inf'))

    def detect_lane_lines(self, image):
        # Convert to grayscale and apply edge detection
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)

        # Define a trapezoidal region of interest (ROI)
        height, width = edges.shape
        mask = np.zeros_like(edges)
        polygon = np.array([[
            (width * 0.1, height),  # Bottom-left
            (width * 0.4, height * 0.6),  # Top-left
            (width * 0.6, height * 0.6),  # Top-right
            (width * 0.9, height)  # Bottom-right
        ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        masked_edges = cv2.bitwise_and(edges, mask)

        # Detect lines using Hough Transform
        lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, threshold=100, minLineLength=50, maxLineGap=50)
        return lines

    def fill_lane_area(self, image, lines):
        if lines is None:
            return image

        left_lines = []
        right_lines = []
        height, width, _ = image.shape
        img_center = width // 2

        # Separate lines into left and right based on their slope and position
        for line in lines:
            # Unpack the line points correctly
            for x1, y1, x2, y2 in line:
                slope = (y2 - y1) / (x2 - x1) if x2 != x1 else float('inf')
                if slope < 0 and x1 < img_center and x2 < img_center:
                    left_lines.append((x1, y1, x2, y2))
                elif slope > 0 and x1 > img_center and x2 > img_center:
                    right_lines.append((x1, y1, x2, y2))

        # Fit a line to the left and right lines
        left_fit_line = self.average_slope_intercept(left_lines, height)
        right_fit_line = self.average_slope_intercept(right_lines, height)

        if left_fit_line is not None and right_fit_line is not None:
            # Define the area between the left and right lanes
            left_x1, left_y1, left_x2, left_y2 = left_fit_line
            right_x1, right_y1, right_x2, right_y2 = right_fit_line

            lane_polygon = np.array([[
                (left_x1, left_y1),
                (right_x1, right_y1),
                (right_x2, right_y2),
                (left_x2, left_y2)
            ]], np.int32)

            # Draw the filled polygon representing the lane area
            cv2.fillPoly(image, lane_polygon, (0, 255, 0))

        return image

    def average_slope_intercept(self, lines, height):
        if not lines:
            return None

        x_coords, y_coords = [], []
        for line in lines:
            x1, y1, x2, y2 = line  # Ensure correct unpacking of line coordinates
            x_coords.extend([x1, x2])
            y_coords.extend([y1, y2])

        if not x_coords:
            return None

        # Fit a line to the collected points
        poly = np.polyfit(y_coords, x_coords, 1)  # Fit a line to the points
        y1 = height
        y2 = int(height * 0.6)
        x1 = int(poly[0] * y1 + poly[1])
        x2 = int(poly[0] * y2 + poly[1])

        return x1, y1, x2, y2

    def publish_image_with_bounding_boxes(self):
        if self.color_image is None:
            return

        # Draw bounding boxes and labels on the image
        image = self.color_image.copy()
        for class_name, confidence, distance, (x1, y1, x2, y2) in self.detections:
            label = f"{class_name} {confidence:.2f} {distance:.2f}m"
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # Detect lane lines and fill the lane area
        lines = self.detect_lane_lines(self.color_image)
        image = self.fill_lane_area(image, lines)

        # Convert and publish the image
        image_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        self.image_teleop_publisher.publish(image_msg)


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

