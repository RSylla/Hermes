import cv2
import numpy as np

# Path to the test image
test_image_path = "/home/raivo/Hermes/hermes_ws/src/object_detection/object_detection/Lane-detection-test-image.png"

# Load the test image
image = cv2.imread(test_image_path)
if image is None:
    print(f"Failed to load test image from {test_image_path}")
    exit(1)

# Function to detect lane lines
def detect_lane_lines(image):
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

# Function to fill the area between lane lines
def fill_lane_area(image, lines):
    if lines is None:
        return image

    left_lines = []
    right_lines = []
    height, width, _ = image.shape
    img_center = width // 2

    # Separate lines into left and right based on their slope and position
    for line in lines:
        for x1, y1, x2, y2 in line:
            slope = (y2 - y1) / (x2 - x1) if x2 != x1 else float('inf')
            if slope < 0 and x1 < img_center and x2 < img_center:
                left_lines.append((x1, y1, x2, y2))
            elif slope > 0 and x1 > img_center and x2 > img_center:
                right_lines.append((x1, y1, x2, y2))

    # Fit a line to the left and right lines
    left_fit_line = average_slope_intercept(left_lines, height)
    right_fit_line = average_slope_intercept(right_lines, height)

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

# Function to average line slopes and intercepts for smoother lane lines
def average_slope_intercept(lines, height):
    if not lines:
        return None

    x_coords, y_coords = [], []
    for line in lines:
        x1, y1, x2, y2 = line
        x_coords.extend([x1, x2])
        y_coords.extend([y1, y2])

    if not x_coords:
        return None

    # Fit a line to the collected points
    poly = np.polyfit(y_coords, x_coords, 1)
    y1 = height
    y2 = int(height * 0.6)
    x1 = int(poly[0] * y1 + poly[1])
    x2 = int(poly[0] * y2 + poly[1])

    return x1, y1, x2, y2

# Perform lane detection and fill the detected lane area
lines = detect_lane_lines(image)
image_with_lanes = fill_lane_area(image, lines)

# Display the resulting image with detected lanes
cv2.imshow('Lane Detection', image_with_lanes)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Save the output image if needed
output_image_path = "/home/raivo/Hermes/hermes_ws/src/object_detection/object_detection/lane_detection_output.png"
cv2.imwrite(output_image_path, image_with_lanes)
print(f"Lane detection output saved to {output_image_path}")
