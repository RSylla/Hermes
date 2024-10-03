import pyrealsense2 as rs
import numpy as np
import cv2
import gi
import torch
from threading import Thread, Lock
from queue import Queue, Empty
import time

# Initialize GStreamer
gi.require_version('Gst', '1.0')
gi.require_version('GstApp', '1.0')
from gi.repository import Gst, GLib

Gst.init(None)

# ===========================
# Configuration Parameters
# ===========================
COLOR_WIDTH = 1280  # Adjust as needed
COLOR_HEIGHT = 720
COLOR_FPS = 30
UDP_HOST = "10.0.3.160"
UDP_PORT = 5000
QUEUE_SIZE = 5  # Max number of frames to hold in queue

# ===========================
# Initialize RealSense Pipeline
# ===========================
rs_pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, COLOR_WIDTH, COLOR_HEIGHT, rs.format.bgr8, COLOR_FPS)
profile = rs_pipeline.start(config)

# ===========================
# Initialize GStreamer Pipeline
# ===========================
gst_pipeline_str = (
    f"appsrc name=source is-live=true format=GST_FORMAT_TIME ! "
    "videoconvert ! video/x-raw,format=I420 ! "
    "x264enc tune=zerolatency speed-preset=superfast ! "
    "h264parse ! rtph264pay config-interval=1 ! "
    f"udpsink host={UDP_HOST} port={UDP_PORT}"
)

gst_pipeline = Gst.parse_launch(gst_pipeline_str)
appsrc = gst_pipeline.get_by_name("source")

# Set caps once
caps = Gst.Caps.from_string(
    f"video/x-raw,format=BGR,width={COLOR_WIDTH},height={COLOR_HEIGHT},framerate={COLOR_FPS}/1"
)
appsrc.set_property("caps", caps)
appsrc.set_property("format", Gst.Format.TIME)
gst_pipeline.set_state(Gst.State.PLAYING)

# ===========================
# Configure YOLOv5 Model
# ===========================
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
model.to(device)
model.eval()

# ===========================
# Initialize OpenCV Window (Optional)
# ===========================
cv2.namedWindow('Person Detection', cv2.WINDOW_AUTOSIZE)

# ===========================
# Frame Queue and Locks
# ===========================
frame_queue = Queue(maxsize=QUEUE_SIZE)
buffer_lock = Lock()

# ===========================
# Frame Capture Thread
# ===========================
def frame_capture():
    try:
        while True:
            frames = rs_pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            color_image = np.asanyarray(color_frame.get_data())
            if not frame_queue.full():
                frame_queue.put(color_image)
            else:
                # Drop frame if queue is full to maintain real-time processing
                pass
    except Exception as e:
        print(f"Frame capture error: {e}")

# ===========================
# Frame Processing Thread
# ===========================
def frame_processing():
    try:
        while True:
            try:
                color_image = frame_queue.get(timeout=1)
            except Empty:
                continue

            # Convert BGR to RGB
            color_image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

            # Perform person detection using YOLOv5
            results = model(color_image_rgb)

            # Filter results to include only 'person' class (class 0 in COCO)
            detections = results.xyxy[0]  # Bounding boxes
            person_detections = detections[detections[:, -1] == 0]  # Class 0 is 'person'

            # Draw bounding boxes for detected persons
            for *box, conf, cls in person_detections:
                x1, y1, x2, y2 = map(int, box)
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f'Person {conf:.2f}'
                cv2.putText(color_image, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Push the processed frame to GStreamer
            with buffer_lock:
                push_to_gstreamer(color_image)

            # Optionally, display the frame
            cv2.imshow('Person Detection', color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"Frame processing error: {e}")

# ===========================
# GStreamer Buffer Push Function
# ===========================
def push_to_gstreamer(processed_image):
    frame_bytes = processed_image.tobytes()
    buffer = Gst.Buffer.new_allocate(None, len(frame_bytes), None)
    buffer.fill(0, frame_bytes)

    # Set timestamp and duration based on system clock
    timestamp = Gst.util_uint64_scale(time.time(), Gst.SECOND, 1)
    buffer.pts = timestamp
    buffer.duration = Gst.util_uint64_scale(1, Gst.SECOND, COLOR_FPS)

    # Push the buffer
    retval = appsrc.emit("push-buffer", buffer)
    if retval != Gst.FlowReturn.OK:
        print("Failed to push buffer to GStreamer")

# ===========================
# Start Threads
# ===========================
capture_thread = Thread(target=frame_capture, daemon=True)
processing_thread = Thread(target=frame_processing, daemon=True)

capture_thread.start()
processing_thread.start()

# ===========================
# Main Loop
# ===========================
try:
    while True:
        # Keep the main thread alive to allow threads to run
        GLib.MainLoop().get_context().iteration(False)
except KeyboardInterrupt:
    print("Interrupted by user")
finally:
    # Cleanup
    rs_pipeline.stop()
    gst_pipeline.set_state(Gst.State.NULL)
    cv2.destroyAllWindows()
