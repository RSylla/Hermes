import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
from ultralytics import YOLO

class YOLOObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_object_detection_node')
        
        # Initialize YOLO model (e.g., yolov8n.pt)
        self.model = YOLO('src/object_detection/object_detection/yolov11n.pt')  # You can change the model here.

        # Publisher for detected objects
        self.publisher_ = self.create_publisher(String, 'detected_objects', 10)
        self.get_logger().info('YOLO Object Detection Node has started.')

        # Start capturing from the laptop webcam (device 0 is the default camera)
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("Webcam could not be opened.")
        else:
            self.get_logger().info("Webcam is opened.")

    def process_webcam(self):
        # Capture frame-by-frame
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().error("Failed to capture image from webcam.")
            return

        # Run YOLO model on the captured frame
        results = self.model(frame)
        obj_names = results[0].names
        print(obj_names)
        # Process the results and extract object information
        detected_objects = []
        for result in results:
            for detection in result.boxes:                                       
                class_id = int(detection.cls)
                confidence = detection.conf.item()
                bbox = detection.xyxy.numpy()  # Bounding box coordinates
                detected_objects.append(
                    f"Class: {class_id}, Object: {obj_names[class_id]}, Confidence: {confidence:.2f}, BBox: {bbox}"
                )

        # Publish the detected objects as a string message
        detected_objects_str = '\n'.join(detected_objects)
        self.publisher_.publish(String(data=detected_objects_str))

        # Log the detection
        # self.get_logger().info(f"Detected objects: \n{detected_objects_str}")

        # Display the frame with detections (optional, for debugging purposes)
        annotated_frame = results[0].plot()  # Annotate the frame with detection results
        cv2.imshow('YOLO Detections', annotated_frame)

        # Exit on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('Shutting down...')
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = YOLOObjectDetectionNode()

    # Process frames from the webcam in a loop
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)  # To allow ROS to handle callbacks
        node.process_webcam()  # Process webcam frames

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
