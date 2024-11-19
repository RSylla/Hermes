import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from .lane_detector import LaneDetector

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
        
        # Lane Detection
        # self.lane_detector = LaneDetector(alpha=0.3, path_position=0.5, smooth_window=5)
        
        # Timers for detection and image publishing
        self.create_timer(1.0 / 3.0, self.perform_detection)  # 3 Hz detection
        self.create_timer(1.0 / 15.0, self.publish_image_with_bounding_boxes)  # 15 Hz image publishing

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def image_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # self.color_image = self.lane_detector.detect_and_fill_lanes(color_image)
        
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
        

    def publish_image_with_bounding_boxes(self):
        if self.color_image is None:
            return

        # Draw bounding boxes and labels on the image
        image = self.color_image.copy()
        for class_name, confidence, distance, (x1, y1, x2, y2) in self.detections:
            label = f"{class_name} {confidence:.2f} {distance:.2f}m"
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

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

