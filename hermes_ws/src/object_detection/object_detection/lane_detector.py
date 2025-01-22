import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math

class LaneFollower(Node):
    def __init__(self):
        super().__init__('lane_follower')

        self.declare_parameter('image_in', '/camera/image_raw')
        self.image_topic = self.get_parameter('image_in').get_parameter_value().string_value
        self.get_logger().info(f"Subscribing to: {self.image_topic}")

        # Set symmetric angular speed limits
        self.declare_parameter('max_angular_speed', 0.2)
        self.declare_parameter('min_angular_speed', -0.2)
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.min_angular_speed = self.get_parameter('min_angular_speed').get_parameter_value().double_value

        # Fixed linear speed (when line is visible)
        self.fixed_linear_speed = 0.3

        self.image_sub = self.create_subscription(Image, self.image_topic, self.callback, 10)
        self.line_pub = self.create_publisher(Point, 'detected_line', 1)
        self.error_pub = self.create_publisher(Point, 'line_error', 1)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.bridge = CvBridge()

        self.desired_offset = 200.0  

        # HSV threshold values
        self.h_min = 2
        self.h_max = 38
        self.s_min = 80
        self.s_max = 255
        self.v_min = 145
        self.v_max = 217
        self.kernel_size = 6
        self.canny_low = 50
        self.canny_high = 150
        self.hough_threshold = 50
        self.min_line_length = 0
        self.max_line_gap = 123

        # PID gains
        # Try lowering Kp if you see too much saturation
        self.Kp = 0.0032
        self.Ki = 0.0
        self.Kd = 0.0

        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = self.get_clock().now()

        # Remember last known steering direction
        self.last_known_steering = 0.0
        self.line_detected_last = False

        # Deadband threshold: if error is within this range, treat it as zero
        self.deadband_threshold = 20.0

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (self.h_min, self.s_min, self.v_min), (self.h_max, self.s_max, self.v_max))

        # Morphological operations
        kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)
        mask_morph = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask_morph = cv2.morphologyEx(mask_morph, cv2.MORPH_DILATE, kernel)

        # Canny edges
        edges = cv2.Canny(mask_morph, self.canny_low, self.canny_high)

        # Hough line transform
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, self.hough_threshold,
                                minLineLength=self.min_line_length, maxLineGap=self.max_line_gap)

        height, width = cv_image.shape[:2]
        center_x = width / 2.0

        chosen_line = None
        if lines is not None:
            # Choose the rightmost line
            max_x_pos = -1
            for line in lines:
                x1, y1, x2, y2 = line[0]
                mid_x = (x1 + x2) / 2.0
                if mid_x > max_x_pos:
                    max_x_pos = mid_x
                    chosen_line = (x1, y1, x2, y2)

        line_image = cv_image.copy()

        # Draw desired line position
        start_point = (int(center_x - self.desired_offset + 100), 0)
        end_point = (int(center_x + self.desired_offset + 120), height)
        cv2.line(line_image, start_point, end_point, (255, 0, 0), 2)

        point_out = Point(x=0.0, y=0.0, z=0.0)
        error_point = Point(x=0.0, y=0.0, z=0.0)

        if chosen_line:
            self.line_detected_last = True
            x1, y1, x2, y2 = chosen_line
            mid_x = (x1 + x2) / 2.0
            mid_y = (y1 + y2) / 2.0
            angle = math.atan2((y2 - y1), (x2 - x1))
            offset_x = mid_x - center_x
            offset_y = mid_y - (height / 2.0)
            point_out.x = float(offset_x)
            point_out.y = float(offset_y)
            point_out.z = float(angle)

            cv2.line(line_image, (x1, y1), (x2, y2), (0,255,0), 2)

            # Compute error based on desired_offset
            error = offset_x - self.desired_offset
            # Apply deadband
            if abs(error) < self.deadband_threshold:
                error = 0.0
            error_point.x = float(error)

            # Publish line info and error
            self.line_pub.publish(point_out)
            self.error_pub.publish(error_point)

            # PID computation
            now = self.get_clock().now()
            dt = (now - self.last_time).nanoseconds / 1e9
            self.last_time = now
            if dt <= 0:
                dt = 0.01

            self.integral += error * dt
            derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
            self.previous_error = error

            # Invert sign so positive error -> negative turn (line on right, turn right)
            steering_cmd = -(self.Kp * error + self.Ki * self.integral + self.Kd * derivative)

            # Small deadband on steering itself
            if abs(steering_cmd) < 0.01:
                steering_cmd = 0.0

            # Clamp steering_cmd
            steering_cmd = max(self.min_angular_speed, min(self.max_angular_speed, steering_cmd))

            # Update last known steering
            self.last_known_steering = steering_cmd

            # Fixed linear speed
            linear_speed = self.fixed_linear_speed

            # Publish cmd_vel
            twist_msg = Twist()
            twist_msg.linear.x = linear_speed
            twist_msg.angular.z = steering_cmd
            self.cmd_vel_pub.publish(twist_msg)

            self.get_logger().info(
                f"Detected line. Error: {error:.4f}, Steering: {steering_cmd:.4f}, "
                f"cmd_vel linear: {twist_msg.linear.x:.2f}, angular: {twist_msg.angular.z:.2f}"
            )

        else:
            # No line detected
            self.get_logger().info("No line detected.")
            if self.line_detected_last:
                # Continue turning in the last known direction
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                search_steering = self.last_known_steering
                # If last known steering was zero, pick a default direction to search
                if abs(search_steering) < 0.01:
                    search_steering = 0.1  
                # Clamp to angular limits
                search_steering = max(self.min_angular_speed, min(self.max_angular_speed, search_steering))
                twist_msg.angular.z = search_steering
                self.cmd_vel_pub.publish(twist_msg)
                self.get_logger().info(
                    f"Searching for line. Continuing turn: {search_steering:.4f}, "
                    f"cmd_vel linear: {twist_msg.linear.x:.2f}, angular: {twist_msg.angular.z:.2f}"
                )
            else:
                # Never detected a line yet or start-up
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                self.cmd_vel_pub.publish(twist_msg)
                self.get_logger().info(
                    f"Waiting for line. cmd_vel linear: {twist_msg.linear.x:.2f}, angular: {twist_msg.angular.z:.2f}"
                )

        cv2.imshow("Final View", line_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    detect_line = LaneFollower()
    try:
        while rclpy.ok():
            rclpy.spin_once(detect_line, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    detect_line.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


#import rclpy
#from rclpy.node import Node
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge
#import cv2
#import numpy as np
#from collections import deque
#
#class LineDetectionNode(Node):
#    def __init__(self):
#        super().__init__('line_detection_node')
#        
#        self.subscription = self.create_subscription(
#            Image,
#            '/camera/image_raw',
#            self.image_callback,
#            10
#        )
#
#        self.bridge = CvBridge()
#
#        # Hyperparameters
#        self.path_offset = self.declare_parameter("path_offset", 0).value
#        self.desired_distance = self.declare_parameter("desired_distance", 250).value
#        self.deviations = deque([0 for i in range(10)], 10)
#        self.current_deviation = sum(self.deviations) / len(self.deviations)  # To store the deviation from the desired distance
#        
#        self.get_logger().info("Line Detection Node Initialized")
#
#    def calculate_deviation(self, line_x):
#        path_center_x = self.get_path_center()
#        return abs(line_x - path_center_x) - self.desired_distance
#
#    def get_path_center(self):
#        # Center of the image adjusted by path offset
#        return self.width // 2 - self.path_offset
#
#    def image_callback(self, msg):
#        try:
#            
#            # Convert ROS Image message to OpenCV format
#            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#
#            # Convert to HSV to isolate yellow color
#            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
#
#            # Define range for yellow color
#            lower_yellow = np.array([20, 100, 100])
#            upper_yellow = np.array([30, 255, 255])
#
#            # Create a mask for yellow color
#            yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
#
#            # Apply the mask to the grayscale image for further processing
#            masked_image = cv2.bitwise_and(cv_image, cv_image, mask=yellow_mask)
#
#            # Convert the masked image to grayscale
#            gray_image = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)
#
#            # Get image dimensions
#            self.height, self.width = gray_image.shape
#
#            # Mask lower half of the image (ROI)
#            roi_height_start = self.height // 2
#            roi = gray_image[roi_height_start:, :]
#
#            # Detect edges (Canny Edge Detection)
#            edges = cv2.Canny(roi, 50, 150)
#
#            # Detect lines using Hough Transform
#            lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)
#
#            # Draw detected line and project path
#            path_center_x = self.get_path_center()
#            if lines is not None:
#                for line in lines:
#                    x1, y1, x2, y2 = line[0]
#                    # Draw the detected line
#                    cv2.line(cv_image, (x1, y1 + roi_height_start), (x2, y2 + roi_height_start), (255, 255, 255), 2)
#
#                    # Calculate the line's center x-coordinate
#                    line_center_x = (x1 + x2) // 2
#
#                    # Calculate deviation
#                    self.deviations.append(self.calculate_deviation(line_center_x))
#
#                # Draw the path line at the center of the image with offset
#                path_x1 = path_center_x
#                path_x2 = path_center_x
#                path_y1 = roi_height_start
#                path_y2 = self.height
#                cv2.line(cv_image, (path_x1, path_y1), (path_x2, path_y2), (200, 200, 200), 2)  # Light gray line
#
#            # Show the processed image (for debugging)
#            print(int(self.current_deviation), x1 - path_x1)
#            cv2.imshow("Processed Image with Path", cv_image)
#            cv2.waitKey(1)
#
#        except Exception as e:
#            self.get_logger().error(f"Error processing image: {e}")
#
#
#def main(args=None):
#    rclpy.init(args=args)
#    node = LineDetectionNode()
#    try:
#        rclpy.spin(node)
#    except KeyboardInterrupt:
#        pass
#    finally:
#        node.destroy_node()
#        rclpy.shutdown()
#
#if __name__ == '__main__':
#    main()

#----------------------------------------------------------------------------------------------------------------

# import cv2
# import numpy as np
# from collections import deque
# from std_msgs.msg import Float32

# class LaneDetector:
#     def __init__(self, alpha=0.3, path_position=0.5, smooth_window=5):
#         """
#         Initialize the LaneDetector with an optional transparency level for lane filling.
#         :param alpha: Transparency level for lane filling, between 0 (fully transparent) and 1 (fully opaque).
#         :param path_position: A float (0 to 1) indicating the position of the path between the lanes.
#                               0 means closer to the left line, 1 means closer to the right line.
#         :param smooth_window: The size of the window for smoothing the path.
#         """
#         self.alpha = alpha
#         self.path_position = np.clip(path_position, 0, 1)
#         self.smooth_window = smooth_window
#         self.path_history = deque(maxlen=smooth_window)
#         self.path_deviation_publisher = None  # ROS publisher for path deviation
#         self.left_fit_history = deque(maxlen=smooth_window)
#         self.right_fit_history = deque(maxlen=smooth_window)

#     def set_publisher(self, publisher):
#         """Set the ROS publisher for path deviation."""
#         self.path_deviation_publisher = publisher

#     def detect_and_fill_lanes(self, image):
#         """
#         Detects lane lines in the given image and fills the lane area with a semi-transparent color.
#         :param image: Input image (numpy array).
#         :return: Processed image with lane lines and filled polygon.
#         """
#         lines = self.detect_lane_lines(image)
#         image_with_lanes = self.fill_lane_area(image.copy(), lines)

#         # Compute the path and draw it on the image
#         path_line, deviation = self.calculate_path(image_with_lanes, lines)
#         if path_line is not None:
#             self.draw_path(image_with_lanes, path_line)
#             self.publish_path_deviation(deviation)

#         return image_with_lanes

#     def detect_lane_lines(self, image):
#         """
#         Detects lane lines using Canny edge detection and Hough line transform.
#         :param image: Input image (numpy array).
#         :return: Detected lines as a list of coordinates.
#         """
#         gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#         edges = cv2.Canny(gray, 50, 150, apertureSize=3)

#         height, width = edges.shape
#         mask = np.zeros_like(edges)
        
#         # Full-width but limited-height polygon for the region of interest (ROI)
#         polygon = np.array([[
#             (0, height),
#             (0, height * 0.5),
#             (width, height * 0.5),
#             (width, height)
#         ]], np.int32)
#         cv2.fillPoly(mask, polygon, 255)
#         masked_edges = cv2.bitwise_and(edges, mask)

#         # Detect lines using Hough Transform with tweaked parameters for better line selection
#         lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, threshold=30, minLineLength=50, maxLineGap=80)
#         return lines

#     def fill_lane_area(self, image, lines):
#         """
#         Fills the area between detected lane lines with a semi-transparent polygon.
#         :param image: Input image (numpy array).
#         :param lines: Detected lane lines.
#         :return: Image with the filled lane area.
#         """
#         if lines is None:
#             return image

#         left_points = []
#         right_points = []
#         height, width, _ = image.shape
#         img_center = width // 2

#         # Separate line points into left and right based on their position and slope
#         for line in lines:
#             for x1, y1, x2, y2 in line:
#                 slope = (y2 - y1) / (x2 - x1) if x2 != x1 else float('inf')
#                 if slope < 0 and x1 < img_center and x2 < img_center:
#                     left_points.extend([(x1, y1), (x2, y2)])
#                 elif slope > 0 and x1 > img_center and x2 > img_center:
#                     right_points.extend([(x1, y1), (x2, y2)])

#         # Fit a polynomial to the left and right lane points
#         left_fit = self.fit_polynomial(left_points, height)
#         right_fit = self.fit_polynomial(right_points, height)

#         if left_fit is not None and right_fit is not None:
#             # Create points for the left and right curves
#             left_curve = [(int(left_fit[0](y)), y) for y in range(height, int(height * 0.5), -1)]
#             right_curve = [(int(right_fit[0](y)), y) for y in range(height, int(height * 0.5), -1)]

#             # Create the polygon points
#             lane_polygon = np.array(left_curve + right_curve[::-1], np.int32)

#             # Create a blank image for the overlay
#             overlay = image.copy()

#             # Draw the filled polygon on the overlay
#             cv2.fillPoly(overlay, [lane_polygon], (0, 255, 0))

#             # Blend the overlay with the original image using the specified alpha for transparency
#             image = cv2.addWeighted(overlay, self.alpha, image, 1 - self.alpha, 0)

#         return image

#     def fit_polynomial(self, points, height, degree=2):
#         """
#         Fits a polynomial to the given points.
#         :param points: List of (x, y) points.
#         :param height: Height of the image.
#         :param degree: Degree of the polynomial to fit (default is 2 for quadratic).
#         :return: A polynomial function representing the fitted curve.
#         """
#         if len(points) < 5:
#             # Too few points for a good polynomial fit; fall back to a straight line
#             degree = 1

#         x_coords, y_coords = zip(*points) if points else ([], [])

#         if not x_coords or len(x_coords) < 2:
#             return None

#         # Fit a polynomial to the points with the specified degree
#         poly = np.polyfit(y_coords, x_coords, degree)
#         poly_func = np.poly1d(poly)

#         return poly_func, poly

#     def calculate_path(self, image, lines):
#         """
#         Calculates a path between the detected lanes based on the desired position between the lanes.
#         :param image: Input image (numpy array).
#         :param lines: Detected lane lines.
#         :return: The path line as a list of (x, y) coordinates and its deviation from the center.
#         """
#         height, width, _ = image.shape
#         img_center = width // 2

#         # Check if lines are detected
#         if lines is None:
#             # No lines detected; return a default path at the center of the image
#             path_x_at_base = img_center
#             path_deviation = 0  # No deviation since the path is centered
#             path_curve = [(path_x_at_base, y) for y in range(height, int(height * 0.5), -1)]
#             return path_curve, path_deviation

#         left_points = []
#         right_points = []
#         for line in lines:
#             for x1, y1, x2, y2 in line:
#                 slope = (y2 - y1) / (x2 - x1) if x2 != x1 else float('inf')
#                 if slope < 0:
#                     left_points.extend([(x1, y1), (x2, y2)])
#                 elif slope > 0:
#                     right_points.extend([(x1, y1), (x2, y2)])

#         left_fit = self.fit_polynomial(left_points, height)
#         right_fit = self.fit_polynomial(right_points, height)

#         if left_fit is None or right_fit is None:
#             # If we can't fit either lane, return a default path at the center
#             path_x_at_base = img_center
#             path_deviation = 0
#             path_curve = [(path_x_at_base, y) for y in range(height, int(height * 0.5), -1)]
#             return path_curve, path_deviation

#         # Calculate the path line by averaging the positions between the lanes
#         path_curve = []
#         for y in range(height, int(height * 0.5), -1):
#             left_x = int(left_fit[0](y))
#             right_x = int(right_fit[0](y))
#             path_x = int(left_x * (1 - self.path_position) + right_x * self.path_position)
#             path_curve.append((path_x, y))

#         # Smooth the path using a moving average
#         smooth_path = self.smooth_path(path_curve)
#         path_x_at_base = smooth_path[0][0]
#         path_deviation = img_center - path_x_at_base

#         return smooth_path, path_deviation

#     def smooth_path(self, path):
#         """
#         Smooths the path using a moving average.
#         :param path: List of (x, y) coordinates representing the path.
#         :return: Smoothed path as a list of (x, y) coordinates.
#         """
#         self.path_history.append(path)
#         average_path = np.mean(self.path_history, axis=0).astype(int)
#         return list(map(tuple, average_path))

#     def draw_path(self, image, path):
#         """
#         Draws the calculated path on the image.
#         :param image: Input image (numpy array).
#         :param path: Path as a list of (x, y) coordinates.
#         """
#         for i in range(len(path) - 1):
#             cv2.line(image, path[i], path[i + 1], (255, 0, 0), 2)  # Draw path line in blue

#     def publish_path_deviation(self, deviation):
#         """
#         Publishes the path deviation to the 'path_deviation' topic.
#         :param deviation: The deviation of the path from the center of the image.
#         """
#         if self.path_deviation_publisher:
#             deviation_msg = Float32()
#             deviation_msg.data = float(deviation)
#             self.path_deviation_publisher.publish(deviation_msg)