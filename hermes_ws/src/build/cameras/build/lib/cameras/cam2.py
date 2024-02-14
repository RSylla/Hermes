import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject

# Initialize GStreamer
Gst.init(None)

def main():
    # Define the pipeline
    pipeline = Gst.parse_launch(
        "v4l2src ! image/jpeg,width=1280,height=720,framerate=30/1 ! "
        "jpegdec ! videoconvert ! x264enc bitrate=500 tune=zerolatency ! "
        "rtph264pay ! udpsink host=10.0.3.183 port=5001"
    )

    # Set the pipeline to playing
    pipeline.set_state(Gst.State.PLAYING)

    # Wait until error or EOS
    bus = pipeline.get_bus()
    while True:
        message = bus.timed_pop_filtered(
            Gst.CLOCK_TIME_NONE,
            Gst.MessageType.ERROR | Gst.MessageType.EOS
        )

        if message:
            if message.type == Gst.MessageType.ERROR:
                err, debug = message.parse_error()
                print("Error:", err, debug)
            break

    # Free resources
    pipeline.set_state(Gst.State.NULL)

if __name__ == "__main__":
    main()