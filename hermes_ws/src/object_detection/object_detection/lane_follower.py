import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

def nothing(x):
    pass

class LaneFollower(Node):
    def __init__(self):
        super().__init__('lane_follower')

        # Declare parameters
        self.declare_parameter('tuning_mode', False)
        self.declare_parameter('image_in', '/camera/image_raw')

        # Get parameters
        self.tuning_mode = self.get_parameter('tuning_mode').get_parameter_value().bool_value
        self.image_topic = self.get_parameter('image_in').get_parameter_value().string_value

        self.get_logger().info(f"Subscribing to: {self.image_topic}")
        self.image_sub = self.create_subscription(Image, self.image_topic, self.callback, 10)
        self.line_pub = self.create_publisher(Point, 'detected_line', 1)
        self.bridge = CvBridge()

        # Initial HSV values for white line detection:
        self.h_min = 0
        self.h_max = 180
        self.s_min = 0
        self.s_max = 60
        self.v_min = 200
        self.v_max = 255

        # Create windows and trackbars if tuning mode is enabled
        if self.tuning_mode:
            cv2.namedWindow("Tuning", cv2.WINDOW_NORMAL)
            cv2.createTrackbar("H Min", "Tuning", self.h_min, 180, nothing)
            cv2.createTrackbar("H Max", "Tuning", self.h_max, 180, nothing)
            cv2.createTrackbar("S Min", "Tuning", self.s_min, 255, nothing)
            cv2.createTrackbar("S Max", "Tuning", self.s_max, 255, nothing)
            cv2.createTrackbar("V Min", "Tuning", self.v_min, 255, nothing)
            cv2.createTrackbar("V Max", "Tuning", self.v_max, 255, nothing)

        self.last_image = None  # We store the last processed image to display it

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        # If tuning mode, read trackbar values
        if self.tuning_mode:
            self.h_min = cv2.getTrackbarPos("H Min", "Tuning")
            self.h_max = cv2.getTrackbarPos("H Max", "Tuning")
            self.s_min = cv2.getTrackbarPos("S Min", "Tuning")
            self.s_max = cv2.getTrackbarPos("S Max", "Tuning")
            self.v_min = cv2.getTrackbarPos("V Min", "Tuning")
            self.v_max = cv2.getTrackbarPos("V Max", "Tuning")

        # Convert to HSV and threshold
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (self.h_min, self.s_min, self.v_min),
                                 (self.h_max, self.s_max, self.v_max))

        # Morphological operations to clean up the mask
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

        # Edge detection
        edges = cv2.Canny(mask, 50, 150, apertureSize=3)

        # Hough line transform
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength=50, maxLineGap=50)

        point_out = Point(x=0.0, y=0.0, z=0.0)
        height, width = cv_image.shape[:2]
        center_x = width / 2.0
        center_y = height / 2.0

        if lines is not None:
            # Choose the rightmost line (just an example heuristic)
            max_x_pos = -1
            chosen_line = None
            for line in lines:
                x1, y1, x2, y2 = line[0]
                mid_x = (x1 + x2) / 2.0
                if mid_x > max_x_pos:
                    max_x_pos = mid_x
                    chosen_line = (x1, y1, x2, y2)

            if chosen_line:
                x1, y1, x2, y2 = chosen_line
                # Draw the chosen line
                cv2.line(cv_image, (x1,y1), (x2,y2), (0,255,0), 2)

                # Compute midpoint and angle
                mid_x = (x1 + x2) / 2.0
                mid_y = (y1 + y2) / 2.0
                angle = np.arctan2((y2 - y1), (x2 - x1))

                # Offset from center
                offset_x = mid_x - center_x
                offset_y = mid_y - center_y

                point_out.x = float(offset_x)
                point_out.y = float(offset_y)
                point_out.z = float(angle)

        # Publish line info
        self.line_pub.publish(point_out)

        # Show images
        # Show mask and edges if you like
        if self.tuning_mode:
            cv2.imshow("Tuning", mask)
        cv2.imshow("Output", cv_image)
        cv2.waitKey(1)

        self.last_image = cv_image

def main(args=None):
    rclpy.init(args=args)
    detect_line = LaneFollower()

    # We spin_once so that we can update the GUI each iteration
    try:
        while rclpy.ok():
            rclpy.spin_once(detect_line, timeout_sec=0.1)
            # cv2.waitKey(1) is called in the callback, but if needed, you can also call it here.
            # This loop ensures the node keeps processing images and trackbar updates.
    except KeyboardInterrupt:
        pass

    detect_line.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
