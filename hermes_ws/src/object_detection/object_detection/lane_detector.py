

import cv2
import numpy as np
from collections import deque
from std_msgs.msg import Float32

class LaneDetector:
    def __init__(self, alpha=0.3, path_position=0.5, smooth_window=5):
        """
        Initialize the LaneDetector with an optional transparency level for lane filling.
        :param alpha: Transparency level for lane filling, between 0 (fully transparent) and 1 (fully opaque).
        :param path_position: A float (0 to 1) indicating the position of the path between the lanes.
                              0 means closer to the left line, 1 means closer to the right line.
        :param smooth_window: The size of the window for smoothing the path.
        """
        self.alpha = alpha
        self.path_position = np.clip(path_position, 0, 1)
        self.smooth_window = smooth_window
        self.path_history = deque(maxlen=smooth_window)
        self.path_deviation_publisher = None  # ROS publisher for path deviation
        self.left_fit_history = deque(maxlen=smooth_window)
        self.right_fit_history = deque(maxlen=smooth_window)

    def set_publisher(self, publisher):
        """Set the ROS publisher for path deviation."""
        self.path_deviation_publisher = publisher

    def detect_and_fill_lanes(self, image):
        """
        Detects lane lines in the given image and fills the lane area with a semi-transparent color.
        :param image: Input image (numpy array).
        :return: Processed image with lane lines and filled polygon.
        """
        lines = self.detect_lane_lines(image)
        image_with_lanes = self.fill_lane_area(image.copy(), lines)

        # Compute the path and draw it on the image
        path_line, deviation = self.calculate_path(image_with_lanes, lines)
        if path_line is not None:
            self.draw_path(image_with_lanes, path_line)
            self.publish_path_deviation(deviation)

        return image_with_lanes

    def detect_lane_lines(self, image):
        """
        Detects lane lines using Canny edge detection and Hough line transform.
        :param image: Input image (numpy array).
        :return: Detected lines as a list of coordinates.
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)

        height, width = edges.shape
        mask = np.zeros_like(edges)
        
        # Full-width but limited-height polygon for the region of interest (ROI)
        polygon = np.array([[
            (0, height),
            (0, height * 0.5),
            (width, height * 0.5),
            (width, height)
        ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        masked_edges = cv2.bitwise_and(edges, mask)

        # Detect lines using Hough Transform with tweaked parameters for better line selection
        lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, threshold=30, minLineLength=50, maxLineGap=80)
        return lines

    def fill_lane_area(self, image, lines):
        """
        Fills the area between detected lane lines with a semi-transparent polygon.
        :param image: Input image (numpy array).
        :param lines: Detected lane lines.
        :return: Image with the filled lane area.
        """
        if lines is None:
            return image

        left_points = []
        right_points = []
        height, width, _ = image.shape
        img_center = width // 2

        # Separate line points into left and right based on their position and slope
        for line in lines:
            for x1, y1, x2, y2 in line:
                slope = (y2 - y1) / (x2 - x1) if x2 != x1 else float('inf')
                if slope < 0 and x1 < img_center and x2 < img_center:
                    left_points.extend([(x1, y1), (x2, y2)])
                elif slope > 0 and x1 > img_center and x2 > img_center:
                    right_points.extend([(x1, y1), (x2, y2)])

        # Fit a polynomial to the left and right lane points
        left_fit = self.fit_polynomial(left_points, height)
        right_fit = self.fit_polynomial(right_points, height)

        if left_fit is not None and right_fit is not None:
            # Create points for the left and right curves
            left_curve = [(int(left_fit[0](y)), y) for y in range(height, int(height * 0.5), -1)]
            right_curve = [(int(right_fit[0](y)), y) for y in range(height, int(height * 0.5), -1)]

            # Create the polygon points
            lane_polygon = np.array(left_curve + right_curve[::-1], np.int32)

            # Create a blank image for the overlay
            overlay = image.copy()

            # Draw the filled polygon on the overlay
            cv2.fillPoly(overlay, [lane_polygon], (0, 255, 0))

            # Blend the overlay with the original image using the specified alpha for transparency
            image = cv2.addWeighted(overlay, self.alpha, image, 1 - self.alpha, 0)

        return image

    def fit_polynomial(self, points, height, degree=2):
        """
        Fits a polynomial to the given points.
        :param points: List of (x, y) points.
        :param height: Height of the image.
        :param degree: Degree of the polynomial to fit (default is 2 for quadratic).
        :return: A polynomial function representing the fitted curve.
        """
        if len(points) < 5:
            # Too few points for a good polynomial fit; fall back to a straight line
            degree = 1

        x_coords, y_coords = zip(*points) if points else ([], [])

        if not x_coords or len(x_coords) < 2:
            return None

        # Fit a polynomial to the points with the specified degree
        poly = np.polyfit(y_coords, x_coords, degree)
        poly_func = np.poly1d(poly)

        return poly_func, poly

    def calculate_path(self, image, lines):
        """
        Calculates a path between the detected lanes based on the desired position between the lanes.
        :param image: Input image (numpy array).
        :param lines: Detected lane lines.
        :return: The path line as a list of (x, y) coordinates and its deviation from the center.
        """
        height, width, _ = image.shape
        img_center = width // 2

        # Check if lines are detected
        if lines is None:
            # No lines detected; return a default path at the center of the image
            path_x_at_base = img_center
            path_deviation = 0  # No deviation since the path is centered
            path_curve = [(path_x_at_base, y) for y in range(height, int(height * 0.5), -1)]
            return path_curve, path_deviation

        left_points = []
        right_points = []
        for line in lines:
            for x1, y1, x2, y2 in line:
                slope = (y2 - y1) / (x2 - x1) if x2 != x1 else float('inf')
                if slope < 0:
                    left_points.extend([(x1, y1), (x2, y2)])
                elif slope > 0:
                    right_points.extend([(x1, y1), (x2, y2)])

        left_fit = self.fit_polynomial(left_points, height)
        right_fit = self.fit_polynomial(right_points, height)

        if left_fit is None or right_fit is None:
            # If we can't fit either lane, return a default path at the center
            path_x_at_base = img_center
            path_deviation = 0
            path_curve = [(path_x_at_base, y) for y in range(height, int(height * 0.5), -1)]
            return path_curve, path_deviation

        # Calculate the path line by averaging the positions between the lanes
        path_curve = []
        for y in range(height, int(height * 0.5), -1):
            left_x = int(left_fit[0](y))
            right_x = int(right_fit[0](y))
            path_x = int(left_x * (1 - self.path_position) + right_x * self.path_position)
            path_curve.append((path_x, y))

        # Smooth the path using a moving average
        smooth_path = self.smooth_path(path_curve)
        path_x_at_base = smooth_path[0][0]
        path_deviation = img_center - path_x_at_base

        return smooth_path, path_deviation

    def smooth_path(self, path):
        """
        Smooths the path using a moving average.
        :param path: List of (x, y) coordinates representing the path.
        :return: Smoothed path as a list of (x, y) coordinates.
        """
        self.path_history.append(path)
        average_path = np.mean(self.path_history, axis=0).astype(int)
        return list(map(tuple, average_path))

    def draw_path(self, image, path):
        """
        Draws the calculated path on the image.
        :param image: Input image (numpy array).
        :param path: Path as a list of (x, y) coordinates.
        """
        for i in range(len(path) - 1):
            cv2.line(image, path[i], path[i + 1], (255, 0, 0), 2)  # Draw path line in blue

    def publish_path_deviation(self, deviation):
        """
        Publishes the path deviation to the 'path_deviation' topic.
        :param deviation: The deviation of the path from the center of the image.
        """
        if self.path_deviation_publisher:
            deviation_msg = Float32()
            deviation_msg.data = float(deviation)
            self.path_deviation_publisher.publish(deviation_msg)