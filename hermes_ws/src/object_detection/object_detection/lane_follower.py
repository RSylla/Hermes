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

        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('min_linear_speed', -1.0)
        self.declare_parameter('max_angular_speed', 0.3)
        self.declare_parameter('min_angular_speed', 0.0)

        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.min_linear_speed = self.get_parameter('min_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.min_angular_speed = self.get_parameter('min_angular_speed').get_parameter_value().double_value

        self.image_sub = self.create_subscription(Image, self.image_topic, self.callback, 10)
        self.line_pub = self.create_publisher(Point, 'detected_line', 1)
        self.error_pub = self.create_publisher(Point, 'line_error', 1)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.bridge = CvBridge()

        # Desired offset from the center of the image (in pixels)
        self.desired_offset = 100.0  

        # Threshold values for line detection (adjust as needed)
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
        self.Kp = 0.0001
        self.Ki = 0.0
        self.Kd = 15.0

        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = self.get_clock().now()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        # Convert image to HSV and create a mask based on HSV thresholds
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (self.h_min, self.s_min, self.v_min), (self.h_max, self.s_max, self.v_max))

        # Morphological operations to clean up the mask
        kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)
        mask_morph = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask_morph = cv2.morphologyEx(mask_morph, cv2.MORPH_DILATE, kernel)

        # Canny edge detection
        edges = cv2.Canny(mask_morph, self.canny_low, self.canny_high)

        # Hough line transform to detect lines
        lines = cv2.HoughLinesP(
            edges, 
            1, 
            np.pi/180, 
            self.hough_threshold,
            minLineLength=self.min_line_length, 
            maxLineGap=self.max_line_gap
        )

        height, width = cv_image.shape[:2]
        center_x = width / 2.0

        chosen_line = None
        if lines is not None:
            # Choose the rightmost line by comparing their midpoint x-coordinates
            max_x_pos = -1
            for line in lines:
                x1, y1, x2, y2 = line[0]
                mid_x = (x1 + x2) / 2.0
                if mid_x > max_x_pos:
                    max_x_pos = mid_x
                    chosen_line = (x1, y1, x2, y2)

        line_image = cv_image.copy()

        # Draw a reference line for the desired offset (optional for debugging)
        # desired_line_x = int(center_x + self.desired_offset)
        # cv2.line(line_image, (desired_line_x, 0), (desired_line_x, height), (255, 0, 0), 2)

        # Just drawing a reference cross line for visualization:
        cv2.line(line_image, (320, 240), (320, 480), (255, 0, 0), 2)

        error_point = Point(x=0.0, y=0.0, z=0.0)

        if chosen_line:
            x1, y1, x2, y2 = chosen_line
            mid_x = (x1 + x2) / 2.0
            offset_x = mid_x - center_x

            # Draw the chosen line in green
            cv2.line(line_image, (x1, y1), (x2, y2), (0,255,0), 2)  

            # Compute the error relative to the desired offset
            error = self.desired_offset - offset_x
            error_point.x = float(error)
            print(error)
            self.error_pub.publish(error_point)

            # Compute time delta
            now = self.get_clock().now()
            dt = (now - self.last_time).nanoseconds / 1e9
            self.last_time = now
            if dt <= 0:
                dt = 0.01

            # PID calculations
            self.integral += error * dt
            derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
            self.previous_error = error

            steering_cmd = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

            # Apply deadband
            if abs(steering_cmd) < 0.05:
                steering_cmd = 0.0

            # Clamp steering command
            steering_cmd = max(-self.max_angular_speed, min(self.max_angular_speed, steering_cmd))

            # Scale linear speed based on how sharply we are turning
            linear_speed = self.max_linear_speed# max(0.05, self.max_linear_speed * (1.0 - abs(steering_cmd) / self.max_angular_speed))

            # Publish cmd_vel
            twist_msg = Twist()
            twist_msg.linear.x = linear_speed
            twist_msg.angular.z = steering_cmd
            self.cmd_vel_pub.publish(twist_msg)

        else:
            # No line found, stop
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(twist_msg)

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




# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Point, Twist
# from cv_bridge import CvBridge, CvBridgeError
# import cv2
# import numpy as np
# import math

# class LaneFollower(Node):
#     def __init__(self):
#         super().__init__('lane_follower')

#         self.declare_parameter('image_in', '/camera/image_raw')
#         self.image_topic = self.get_parameter('image_in').get_parameter_value().string_value
#         self.get_logger().info(f"Subscribing to: {self.image_topic}")

#         self.declare_parameter('max_linear_speed', 0.5)
#         self.declare_parameter('min_linear_speed', -1.0)
#         self.declare_parameter('max_angular_speed', 1.0)
#         self.declare_parameter('min_angular_speed', 0.0)

#         self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
#         self.min_linear_speed = self.get_parameter('min_linear_speed').get_parameter_value().double_value
#         self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
#         self.min_angular_speed = self.get_parameter('min_angular_speed').get_parameter_value().double_value

#         self.image_sub = self.create_subscription(Image, self.image_topic, self.callback, 10)
#         self.line_pub = self.create_publisher(Point, 'detected_line', 1)
#         self.error_pub = self.create_publisher(Point, 'line_error', 1)
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

#         self.bridge = CvBridge()

#         self.desired_offset = 250.0  

#         # Must change these values
#         self.h_min = 2
#         self.h_max = 38
#         self.s_min = 80
#         self.s_max = 255
#         self.v_min = 145
#         self.v_max = 217
#         self.kernel_size = 6
#         self.canny_low = 50
#         self.canny_high = 150
#         self.hough_threshold = 50 #18
#         self.min_line_length = 0
#         self.max_line_gap = 123


#         # cv2.namedWindow("Tuner", cv2.WINDOW_NORMAL)
#         # cv2.createTrackbar("H Min", "Tuner", 2, 179, lambda x: x)
#         # cv2.createTrackbar("H Max", "Tuner", 38, 179, lambda x: x)
#         # cv2.createTrackbar("S Min", "Tuner", 80, 255, lambda x: x)
#         # cv2.createTrackbar("S Max", "Tuner", 255, 255, lambda x: x)
#         # cv2.createTrackbar("V Min", "Tuner", 145, 255, lambda x: x)
#         # cv2.createTrackbar("V Max", "Tuner", 217, 255, lambda x: x)
#         # cv2.createTrackbar("Kernel Size", "Tuner", 6, 20, lambda x: x)
#         # cv2.createTrackbar("Canny Low", "Tuner", 50, 255, lambda x: x)
#         # cv2.createTrackbar("Canny High", "Tuner", 150, 255, lambda x: x)
#         # cv2.createTrackbar("Hough Threshold", "Tuner", 1, 200, lambda x: x)
#         # cv2.createTrackbar("Min Line Length", "Tuner", 12, 200, lambda x: x)
#         # cv2.createTrackbar("Max Line Gap", "Tuner", 128, 200, lambda x: x)

#         self.Kp = 50.5005  # Proportional gain
#         self.Ki = 0.0    # Integral gain 
#         self.Kd = 5.0    # Derivative gain 

#         self.previous_error = 0.0
#         self.integral = 0.0
#         self.last_time = self.get_clock().now()

#     def callback(self, data):
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
#         except CvBridgeError as e:
#             self.get_logger().error(f"CV Bridge Error: {e}")
#             return

#         hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
#         mask = cv2.inRange(hsv, (self.h_min, self.s_min, self.v_min), (self.h_max, self.s_max, self.v_max))

#         # Morphological operations
#         kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)
#         mask_morph = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
#         mask_morph = cv2.morphologyEx(mask_morph, cv2.MORPH_DILATE, kernel)

#         # Canny edges
#         edges = cv2.Canny(mask_morph, self.canny_low, self.canny_high)

#         # Hough line transform
#         lines = cv2.HoughLinesP(edges, 1, np.pi/180, self.hough_threshold,
#                                 minLineLength=self.min_line_length, maxLineGap=self.max_line_gap)

#         point_out = Point(x=0.0, y=0.0, z=0.0)
#         error_point = Point(x=0.0, y=0.0, z=0.0)
#         height, width = cv_image.shape[:2]
#         center_x = width / 2.0

#         chosen_line = None
#         if lines is not None:
#             # Choose the rightmost line
#             max_x_pos = -1
#             for line in lines:
#                 x1, y1, x2, y2 = line[0]
#                 mid_x = (x1 + x2) / 2.0
#                 if mid_x > max_x_pos:
#                     max_x_pos = mid_x
#                     chosen_line = (x1, y1, x2, y2)

#         line_image = cv_image.copy()
#         # Draw the desired line position in the image
        
#         # desired_line_x = int(center_x + self.desired_offset)
#         # cv2.line(line_image, (desired_line_x, 0), (desired_line_x, height), (255, 0, 0), 2)  # Blue line
        
#         # start_point = (chosen_line[0] - self.desired_offset, chosen_line[1]) #(int(center_x - self.desired_offset + 100), 0)
#         # end_point = (chosen_line[2] - self.desired_offset, chosen_line[3]) #(int(center_x + self.desired_offset+120), height)
#         cv2.line(line_image, (320, 240), (320, 480), (255, 0, 0), 2)
        
#         offset_x = 0.0
#         errors_list = []
#         if chosen_line:
#             x1, y1, x2, y2 = chosen_line
#             mid_x = (x1 + x2) / 2.0
#             mid_y = (y1 + y2) / 2.0
#             # angle = math.atan2((y2 - y1), (x2 - x1))
#             offset_x = mid_x - center_x
                       
#             # offset_y = mid_y - (height / 2.0)  # vertical offset
#             # point_out.x = float(offset_x)
#             # point_out.y = float(offset_y)
#             # point_out.z = float(angle)

#             cv2.line(line_image, (x1, y1), (x2, y2), (0,255,0), 2)  
            
#             error = offset_x - self.desired_offset
#             errors_list.append(error)
#             if len(errors_list) > 5:
#                 errors_list.pop(0)
            
#             avg_error = sum(errors_list) / len(errors_list)
            
#             error_point.x = float(avg_error)
#             print(error_point)
#             # Publish line info and error
#             # self.line_pub.publish(point_out)
#             self.error_pub.publish(error_point)

#             # Compute PID output
#             now = self.get_clock().now()
#             dt = (now - self.last_time).nanoseconds / 1e9
#             self.last_time = now

#             if dt <= 0:
#                 dt = 0.01 

#             # PID calculations
#             self.integral += error * dt
#             derivative = (error - self.previous_error) / dt if dt > 0 else 0.0

#             steering_cmd = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
#             self.previous_error = error

#             # Apply deadband to steering command
#             if abs(steering_cmd) < 0.05:  # Adjust as necessary
#                 steering_cmd = 0.0

#             # Cap the steering command
#             steering_cmd = max( min(self.max_angular_speed, steering_cmd))

#             # Scale linear speed based on angular speed
#             linear_speed = max(0.05, self.max_linear_speed * (1.0 - abs(steering_cmd) / self.max_angular_speed))

#             # Publish cmd_vel
#             twist_msg = Twist()
#             twist_msg.linear.x = linear_speed
#             twist_msg.angular.z = steering_cmd
#             self.cmd_vel_pub.publish(twist_msg)

#         else:
#             twist_msg = Twist()
#             twist_msg.linear.x = 0.0
#             twist_msg.angular.z = 0.0
#             self.cmd_vel_pub.publish(twist_msg)

#         # mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
#         # edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
#         # mask_morph_bgr = cv2.cvtColor(mask_morph, cv2.COLOR_GRAY2BGR)
#         # top_stack = np.hstack([cv_image, mask_bgr, result])
#         # bottom_stack = np.hstack([edges_bgr, line_image, mask_morph_bgr])
#         # combined = np.vstack([top_stack, bottom_stack])
#         # cv2.imshow("Processed View", combined)

#         cv2.imshow("Final View", line_image)
#         cv2.waitKey(1)

# def main(args=None):
#     rclpy.init(args=args)
#     detect_line = LaneFollower()
#     try:
#         while rclpy.ok():
#             rclpy.spin_once(detect_line, timeout_sec=0.1)
#     except KeyboardInterrupt:
#         pass
#     detect_line.destroy_node()
#     cv2.destroyAllWindows()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
