import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch

class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')
        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None

        # Load YOLO model using torch
        model_path = '/home/hermes/Hermes/hermes_ws/src/object_detection/object_detection/yolov5m_Objects365.pt'
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)
        self.model.to('cuda')
        
        # Define objects to track
        self.target_classes = ['Person', 'Car', 'Traffic cone', 'Bicycle', 'Motorcycle']    

        # Publishers
        self.objects_pub = self.create_publisher(String, 'detected_objects', 10)
        # New publisher for annotated image
        self.image_teleop_pub = self.create_publisher(Image, 'image_teleop', 10)

        # Subscribers
        self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.create_subscription(Image, 'camera/image_depth', self.depth_callback, 10)
        
        # Detection timer
        self.create_timer(0.1, self.detect_objects)  # 10Hz

    def image_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def detect_objects(self):
        if self.color_image is None or self.depth_image is None:
            return

        # Make a copy so we don't overwrite the original (optional)
        annotated_image = self.color_image.copy()

        # Run inference with YOLO
        results = self.model(annotated_image)
        # print(results)
        detected_objects = []

        # Parse detections
        for *xyxy, conf, cls in results.xyxy[0]:
            confidence = conf.item()
            class_name = self.model.names[int(cls.item())]

            if confidence > 0.2 and class_name in self.target_classes:
                # print(f"{class_name}: {confidence:.2f}")
                x1, y1, x2, y2 = map(int, xyxy)
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2

                # Depth in meters (assuming depth_image is in millimeters)
                depth = self.depth_image[center_y, center_x] / 1000.0

                # Prepare text for detection log
                detected_objects.append([class_name, depth, confidence])

                # Draw bounding box
                cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"{class_name} {depth}"
                cv2.putText(
                    annotated_image,
                    label,
                    (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA
                )

        # Publish detections (as text)
        if detected_objects:
            sorted_objects = sorted(detected_objects, key=lambda x: x[1])
            self.get_logger().info(f"Detected objects: {sorted_objects}")
            msg = String()
            # print(str(detected_objects))
            msg.data = str(sorted_objects)
            self.objects_pub.publish(msg)

        # Publish the annotated image
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")
        self.image_teleop_pub.publish(annotated_msg)


def main():
    rclpy.init()
    node = ObjectDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
