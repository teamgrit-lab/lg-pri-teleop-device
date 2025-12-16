import rclpy
from rclpy.node import Node
from lg_teleop_device.abstract_teleop_node import AbstractTeleopNode
from sensor_msgs.msg import Image
import asyncio
import gi
import threading
gi.require_version('Gst', '1.0')
from gi.repository import Gst
import time, json
from std_msgs.msg import String

class TopicMediaNode(AbstractTeleopNode):
    def __init__(self, name='media_node'):
        super().__init__(name)
        self.declare_parameter('topic', '')
        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.declare_parameter('width', '')
        self.width = self.get_parameter('width').get_parameter_value().string_value
        self.declare_parameter('height', '')
        self.height = self.get_parameter('height').get_parameter_value().string_value
        self.pipeline_cmd = f"appsrc name=src is-live=true block=false format=time do-timestamp=true emit-signals=true caps=video/x-raw,format=RGB,framerate=30/1,width={self.width},height={self.height} ! videoconvert ! x264enc bitrate=3000 key-int-max=30 tune=zerolatency speed-preset=1 ! h264parse config-interval=-1 ! video/x-h264, stream-format=byte-stream, alignment=au ! queue leaky=2 ! appsink name=sink sync=false emit-signals=true drop=true max-buffers=5"
        

        
        self.pipeline = Gst.parse_launch(self.pipeline_cmd)
        self.sink = self.pipeline.get_by_name('sink')
        self.src = self.pipeline.get_by_name('src')
        self.sink.connect('new-sample', self.on_new_sample)
        self.pipeline.set_state(Gst.State.PLAYING)
        self.read_timer = None
        self.processing_timer = None
        self.encoding_timer = None
        self.read_time_diff = 0.0
        self.processing_time_diff = 0.0
        self.encoding_time_diff = 0.0

        self.network_time = 0.0

        self.mime_sent = False
        self.last_ping_time = time.time()

        self.suber = self.create_subscription(
            Image,
            self.topic,
            self.callback,
            10
        )

    def callback(self, msg):
        try:
            # 최근 ping이 10초 이상 없으면 인코딩 자체를 건너뛴다.
            if time.time() - self.last_ping_time > 10:
                return

            if self.read_timer is not None:
                now = time.time()
                self.read_time_diff = now - self.read_timer
                self.read_timer = None
            if self.processing_timer is None:
                self.processing_timer = time.time()

            buf_data = bytes(msg.data)
            buf = Gst.Buffer.new_wrapped(buf_data)
            self.src.emit('push-buffer', buf)

            if self.processing_timer is not None:
                now = time.time()
                self.processing_time_diff = now - self.processing_timer
                self.processing_timer = None

            json_msg = {
                "name": self.name,
                "processingTimes_ms": {
                    "sensorRead": self.read_time_diff * 1000,
                    "processing": self.processing_time_diff * 1000,
                },
                "fps": self.fps
            }
            if time.time() - self.network_time > 1.0:
                self.network_time = time.time()
                json_str = json.dumps(json_msg)
                self.network_publisher.publish(String(data=json_str))
                self.fps = 0
        except Exception as e:
            self.get_logger().error(f"[{self.name}] Failed to process message: {e}")

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
        if self.read_timer is None:
            self.read_timer = time.time()
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
    
    async def on_received(self, message):
        """
        Handles the received message from the WebSocket.
        This method should be overridden in subclasses to process the received message.
        :param message: The message received from the WebSocket.
        """
        try:
            if isinstance(message, bytes):
                # Ping은 UTF-8로 들어오므로 그대로 디코딩
                decoded = message.decode("utf-8", errors="ignore")
            else:
                decoded = message

            if isinstance(decoded, str) and decoded.strip().lower() == "ping":
                self.last_ping_time = time.time()
                return

            self.get_logger().info(f"[{self.name}] Received message: {decoded}")
        except Exception as e:
            self.get_logger().error(f"[{self.name}] Failed to process incoming message: {e}")

def main(args=None):
    rclpy.init(args=args)
    Gst.init(None)

    temp_node = Node('temp_node')
    temp_node.declare_parameter('name', 'topic_media_node')
    name = temp_node.get_parameter('name').get_parameter_value().string_value
    temp_node.destroy_node()

    node = TopicMediaNode(name=name)

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
