import pyrealsense2 as rs
import numpy as np
import cv2
import gi

gi.require_version('Gst', '1.0')
gi.require_version('GstApp', '1.0')
from gi.repository import Gst, GstApp, GLib

# Initialize GStreamer
Gst.init(None)

# Configure RealSense pipeline
rs_pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start RealSense streaming
rs_pipeline.start(config)

# Create GStreamer pipeline
gst_pipeline_str = (
    "appsrc name=source is-live=true format=GST_FORMAT_TIME ! "
    "videoconvert ! x265enc tune=zerolatency bitrate=2000 ! "
    "rtph265pay config-interval=1 ! "
    "tcpserversink host=127.0.0.1 port=5000"
)

# Create and start GStreamer pipeline
gst_pipeline = Gst.parse_launch(gst_pipeline_str)
appsrc = gst_pipeline.get_by_name("source")

# Set appsrc properties
caps = Gst.Caps.from_string("video/x-raw,format=BGR,width=1280,height=720,framerate=30/1")
appsrc.set_property("caps", caps)

gst_pipeline.set_state(Gst.State.PLAYING)

try:
    while True:
        # Wait for a coherent pair of frames
        frames = rs_pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())

        # Create GStreamer buffer
        buffer = Gst.Buffer.new_wrapped(color_image.tobytes())

        # Push the buffer to appsrc
        appsrc.emit("push-buffer", buffer)

        # Show images (optional, for debugging)
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', color_image)
        
        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming
    rs_pipeline.stop()
    gst_pipeline.set_state(Gst.State.NULL)
    cv2.destroyAllWindows()


