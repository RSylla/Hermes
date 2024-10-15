import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # ROS2 message type for publishing objects
import pyrealsense2 as rs
import numpy as np
import cv2
import torch

class RealSenseYOLOv5(Node):
    def __init__(self):
        super().__init__('realsense_yolov5')

        # Configure RealSense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()

        # Configure the color and depth streams
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # Start streaming
        self.pipeline.start(config)

        # Load YOLOv5 model with CUDA enabled
        self.model = torch.hub.load('ultralytics/yolov5', 
                                    'custom', 
                                    path="/home/hermes/Hermes/hermes_ws/src/cameras/cameras/yolov5m_Objects365.pt", 
                                    force_reload=True)
        self.model.cuda()  # Use CUDA for faster processing
        self.model.eval()  # Set model to evaluation mode

        # Publisher for detected objects with distance
        self.objects_publisher = self.create_publisher(String, 'objects', 10)

        # Timer to periodically process frames
        self.timer = self.create_timer(0.1, self.process_frames)

    def process_frames(self):
        # Get frames from the RealSense pipeline
        frames = self.pipeline.wait_for_frames()

        # Get the color and depth frames
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return

        # Convert frames to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # Run YOLOv5 object detection on the color image (use CUDA)
        results = self.model(color_image, device='cuda')

        # Extract bounding boxes, labels, confidences, and calculate distances
        detections = []
        for *box, conf, cls in results.xyxy[0].cpu().numpy():  # Format: [x1, y1, x2, y2, confidence, class]
            x1, y1, x2, y2 = map(int, box)
            class_name = self.model.names[int(cls)]
            confidence = conf

            # Calculate the distance from the depth image
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            distance = depth_image[center_y, center_x] * 0.001  # Convert depth from mm to meters

            # Store detection with distance
            detections.append((class_name, confidence, distance, (x1, y1, x2, y2)))

        # Sort detections by nearest distance first
        detections.sort(key=lambda x: x[2])

        # Publish sorted objects to the 'objects' topic
        objects_msg = String()
        objects_data = []
        for class_name, confidence, distance, box in detections:
            objects_data.append(f"{class_name} {confidence:.2f} Distance: {distance:.2f}m")
        objects_msg.data = ', '.join(objects_data)
        self.objects_publisher.publish(objects_msg)

        # Display the color image with bounding boxes
        for class_name, confidence, distance, (x1, y1, x2, y2) in detections:
            # Draw bounding box on color image
            cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f'{class_name} {confidence:.2f} Distance: {distance:.2f}m'
            cv2.putText(color_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        cv2.imshow('YOLOv5 Detection with Distance', color_image)

        # Wait for a key press for 1ms to allow OpenCV to process the display
        if cv2.waitKey(1) == 27:  # Press 'ESC' to exit
            self.pipeline.stop()
            cv2.destroyAllWindows()
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    realsense_yolov5 = RealSenseYOLOv5()

    try:
        rclpy.spin(realsense_yolov5)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the RealSense pipeline
        realsense_yolov5.pipeline.stop()
        realsense_yolov5.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
