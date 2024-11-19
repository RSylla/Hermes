import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from geometry_msgs.msg import Twist
from .lane_detector import LaneDetector  # If you have a lane_detector module

class ObjectDetection(Node):
    def __init__(self):
        super().__init__('objects_detection')
        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.detections = []
        self.traffic_cone_detected = False  # Detection flag
        self.safety_timer = None  # Timer for continuously publishing zero velocities

        # YOLO model path
        model_path = "/home/hermes/Hermes/hermes_ws/src/object_detection/object_detection/yolov11n_self_driving_car.pt"
        self.model = YOLO(model_path).to("cuda")

        # Publishers
        self.objects_publisher = self.create_publisher(String, 'object_detection', 10)
        self.image_teleop_publisher = self.create_publisher(Image, 'image_teleop', 10)
        self.cone_detected_publisher = self.create_publisher(Bool, 'traffic_cone_detected', 10)
        self.cmd_vel_safe_publisher = self.create_publisher(Twist, 'cmd_vel_safe', 10)

        # Subscriptions
        self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.create_subscription(Image, 'camera/image_depth', self.depth_callback, 10)

        # Lane Detection (Uncomment if using)
        # self.lane_detector = LaneDetector(alpha=0.3, path_position=0.5, smooth_window=5)

        # Timers for detection and image publishing
        self.create_timer(1.0 / 3.0, self.perform_detection)  # 3 Hz
        self.create_timer(1.0 / 15.0, self.publish_image_with_bounding_boxes)  # 15 Hz

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def image_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # If using lane detection:
        # self.color_image = self.lane_detector.detect_and_fill_lanes(self.color_image)

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

        # Perform YOLO object detection
        results = self.model.predict(self.color_image)
        self.detections = self.extract_detections(results)

        # Parameters for filtering cones
        max_detection_distance = 3.0  # meters
        min_confidence = 0.5  # Only consider cones with >= 0.5 confidence

        # Filter for traffic cones meeting distance and confidence criteria
        traffic_cone_detections = [
            det for det in self.detections
            if det[0] == 'traffic-cone' and det[1] >= min_confidence and det[2] != -1 and det[2] <= max_detection_distance
        ]

        num_traffic_cones = len(traffic_cone_detections)
        traffic_cone_present = num_traffic_cones > 0

        # If detection state changed, update flag and robot control
        if traffic_cone_present != self.traffic_cone_detected:
            self.traffic_cone_detected = traffic_cone_present
            self.cone_detected_publisher.publish(Bool(data=self.traffic_cone_detected))
            self.get_logger().info(f"Traffic cone detected: {self.traffic_cone_detected}")
            self.control_robot_motion()

        # Publish detection info
        self.publish_detections()

    def control_robot_motion(self):
        if self.traffic_cone_detected:
            # Start publishing zero velocities to cmd_vel_safe if not already
            if self.safety_timer is None:
                self.get_logger().info("Starting safety timer: stopping robot due to traffic cone.")
                self.safety_timer = self.create_timer(0.1, self.publish_zero_velocity)
        else:
            # If no cone, stop publishing zero velocities
            if self.safety_timer is not None:
                self.get_logger().info("Traffic cone no longer detected; stopping safety timer.")
                self.safety_timer.cancel()
                self.safety_timer = None

    def publish_zero_velocity(self):
        # Continuously publish zero velocity to cmd_vel_safe
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_safe_publisher.publish(twist_msg)

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
                        depth_value = self.depth_image[center_y, center_x]
                        if depth_value > 0:
                            distance = depth_value * 0.001  # mm to meters

                detections.append((class_name, confidence, distance, (x1, y1, x2, y2)))

        # Sort by distance if known, else cones without distance at the end
        return sorted(detections, key=lambda x: x[2] if x[2] != -1 else float('inf'))

    def publish_image_with_bounding_boxes(self):
        if self.color_image is None:
            return

        image = self.color_image.copy()
        for class_name, confidence, distance, (x1, y1, x2, y2) in self.detections:
            label = f"{class_name} {confidence:.2f} {distance:.2f}m"
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        # If using lane detection, uncomment:
        # lines = self.detect_lane_lines(self.color_image)
        # image = self.fill_lane_area(image, lines)

        image_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        self.image_teleop_publisher.publish(image_msg)

    # If using lane detection, implement these methods:
    # def detect_lane_lines(self, image):
    #     pass

    # def fill_lane_area(self, image, lines):
    #     return image

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
