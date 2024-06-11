import pyrealsense2 as rs
import numpy as np
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

# Initialize GStreamer
Gst.init(None)

# Configure the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 15)  # Lower resolution here
pipeline.start(config)

# Create GStreamer pipeline
gst_pipeline = Gst.Pipeline.new("realsense-pipeline")
appsrc = Gst.ElementFactory.make("appsrc", "source")
videoconvert = Gst.ElementFactory.make("videoconvert", "converter")
x264enc = Gst.ElementFactory.make("x264enc", "encoder")
rtppay = Gst.ElementFactory.make("rtph264pay", "rtppay")
udpsink = Gst.ElementFactory.make("udpsink", "sink")

# Configure the x264 encoder for lower quality
x264enc.set_property("tune", "zerolatency")
x264enc.set_property("speed-preset", "medium")  # Use a faster preset for lower quality
x264enc.set_property("bitrate", 1000)  # Lower bitrate (e.g., 500 kbit/sec)
x264enc.set_property("quantizer", 30)  # Higher quantizer for lower quality

# Set the host and port for udpsink
udpsink.set_property("host", "10.0.3.183")  # Replace with your destination IP
udpsink.set_property("port", 5000)  # Replace with your desired port

# Set capabilities for appsrc with updated resolution
caps = Gst.Caps.from_string("video/x-raw, format=BGR, width=1280, height=720, framerate=15/1")  # Lower resolution here
appsrc.set_property("caps", caps)
appsrc.set_property("format", Gst.Format.TIME)

# Add and link elements in the pipeline
gst_pipeline.add(appsrc)
gst_pipeline.add(videoconvert)
gst_pipeline.add(x264enc)
gst_pipeline.add(rtppay)
gst_pipeline.add(udpsink)

appsrc.link(videoconvert)
videoconvert.link(x264enc)
x264enc.link(rtppay)
rtppay.link(udpsink)

# Start the GStreamer pipeline
gst_pipeline.set_state(Gst.State.PLAYING)

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        frame_data = np.asanyarray(color_frame.get_data()).tobytes()
        gst_buffer = Gst.Buffer.new_wrapped(frame_data)
        ret = appsrc.emit("push-buffer", gst_buffer)
        if ret != Gst.FlowReturn.OK:
            print("Error in pushing buffer to pipeline")
            break
except KeyboardInterrupt:
    print("\nStopping Stream")
finally:
    pipeline.stop()
    gst_pipeline.set_state(Gst.State.NULL)