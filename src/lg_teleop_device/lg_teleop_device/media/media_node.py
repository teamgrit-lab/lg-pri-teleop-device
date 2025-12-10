import rclpy
from rclpy.node import Node
from lg_teleop_device.abstract_teleop_node import AbstractTeleopNode
import asyncio
import gi
import threading
gi.require_version('Gst', '1.0')
from gi.repository import Gst

class MediaNode(AbstractTeleopNode):
    def __init__(self, name='media_node'):
        super().__init__(name)
        self.declare_parameter('pipeline', None)
        self.pipeline_cmd = self.get_parameter('pipeline').get_parameter_value().string_value

        if not self.pipeline_cmd:
            self.get_logger().error("No pipeline command provided via --ros-args.")
            return
        if "264" in self.pipeline_cmd:
            self.pipeline_cmd = f"{self.pipeline_cmd} ! h264parse config-interval=1 ! video/x-h264, alignment=au, stream-format=byte-stream ! queue leaky=2 ! appsink name=sink drop=true emit-signals=true sync=false max-buffers=3"
        elif "265" in self.pipeline_cmd:
            self.pipeline_cmd = f"{self.pipeline_cmd} ! h265parse config-interval=1 ! video/x-h265, alignment=au, stream-format=byte-stream ! queue leaky=2 ! appsink name=sink drop=true emit-signals=true sync=false max-buffers=3"
        elif "opus" in self.pipeline_cmd:
            self.pipeline_cmd = f"{self.pipeline_cmd} ! opusparse ! queue leaky=2 ! appsink name=sink drop=true emit-signals=true sync=false max-buffers=3"

        self.pipeline = Gst.parse_launch(self.pipeline_cmd)
        self.sink = self.pipeline.get_by_name('sink')
        self.sink.connect('new-sample', self.on_new_sample)
        self.pipeline.set_state(Gst.State.PLAYING)

        self.mime_sent = False

    def create_mime(self):
        """
        Create MIME type based on the pipeline configuration.
        This method is called once to send the MIME type to the WebSocket server.
        """
        if self.mime:
            return self.mime
        return "application/x-unknown"

    def on_new_sample(self, sink):
        """
        Callback for new samples from the appsink.
        This method is called when a new sample is available in the appsink.
        """
        
        sample = sink.emit('pull-sample')
        if not sample:
            return Gst.FlowReturn.ERROR
        buffer = sample.get_buffer()
        data = buffer.extract_dup(0, buffer.get_size())
        self.message.append(data)
        if self.mime_sent:
            return Gst.FlowReturn.OK
        caps = self.sink.get_static_pad("sink").get_current_caps()  # Get caps from the sink pad
        if caps:
            # self.get_logger().info(f"[{self.name}] Extracting MIME type from caps: {caps.to_string()}")
            structure = caps.get_structure(0)  # Get the first structure from caps
            mime_type = structure.get_name()

            if "x-" in mime_type:
                mime_type = mime_type.replace("x-", "")

            width = structure.get_int("width")[1] if structure.has_field("width") else None
            height = structure.get_int("height")[1] if structure.has_field("height") else None
            framerate = structure.get_fraction("framerate")[1] if structure.has_field("framerate") else None

            # Handle Opus-specific fields
            if "opus" in mime_type:
                channels = structure.get_int("channels")[1] if structure.has_field("channels") else None
                rate = structure.get_int("rate")[1] if structure.has_field("rate") else None

                # Format the MIME string for Opus
                self.mime = f"{mime_type};numberOfChannels={channels};sampleRate={rate};codecs=opus"
                self.mime_sent = True
            else:
                # Handle video-specific fields
                profile = structure.get_string("profile") if structure.has_field("profile") else "main"
                level = structure.get_string("level") if structure.has_field("level") else "3.0"
                tier = structure.get_string("tier") if structure.has_field("tier") else "main"  # Default to "main"

                if "h264" in mime_type:
                    profile_map = {
                        "baseline": "42",
                        "constrained-baseline": "42",
                        "main": "4D",
                        "high": "64",
                        "high-10": "6E",
                        "high-10-intra": "6E",
                        "high-4:2:2": "7A",
                        "high-4:2:2-intra": "7A",
                        "high-4:4:4": "F4",
                        "high-4:4:4-intra": "F4"
                    }
                    profile_hex = profile_map.get(profile.lower(), "42")  # Default to baseline
                    codec_prefix = "avc1"
                    tier_flag = "00"  # H.264 doesn't use tier concept, so set to "00" or empty string
                elif "h265" in mime_type:
                    profile_map = {
                        "main": "01",
                        "main-still-picture": "03",
                        "main-intra": "01",
                        "main-444": "04",
                        "main-444-intra": "04",
                        "main-444-still-picture": "04",
                        "main-10": "02",
                        "main-10-intra": "02",
                        "main-422-10": "06",
                        "main-422-10-intra": "06",
                        "main-444-10": "05",
                        "main-444-10-intra": "05",
                        "main-12": "08",
                        "main-12-intra": "08",
                        "main-422-12": "07",
                        "main-422-12-intra": "07",
                        "main-444-12": "09",
                        "main-444-12-intra": "09"
                    }
                    profile_hex = profile_map.get(profile.lower(), "01")  # Default to main
                    codec_prefix = "hev1"
                    tier_flag = "01" if tier.lower() == "high" else "00"  # High tier = 01, Main tier = 00
                else:
                    profile_hex = "00"
                    codec_prefix = "unknown"
                    tier_flag = "00"

                level_hex = f"{int(float(level) * 10):02X}"  # Convert level to hex (e.g., 4.2 -> 2A)

                self.mime = f"{mime_type};width={width};height={height};framerate={framerate};codecs={codec_prefix}.{profile_hex}{tier_flag}{level_hex}"
                self.mime_sent = True
        return Gst.FlowReturn.OK
    
def main(args=None):
    rclpy.init(args=args)
    Gst.init(None)

    temp_node = Node('temp_node')
    temp_node.declare_parameter('name', 'media_node')
    name = temp_node.get_parameter('name').get_parameter_value().string_value
    temp_node.destroy_node()

    node = MediaNode(name=name)

    executor = rclpy.executors.MultiThreadedExecutor()

    def run_asyncio():
        asyncio.run(node.run())

    asyncio_thread = threading.Thread(target=run_asyncio, daemon=True)
    asyncio_thread.start()

    try:
        executor.add_node(node)
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
