import rclpy
from rclpy.node import Node
import websockets
import asyncio
from abc import ABC, abstractmethod
import ssl
from std_msgs.msg import String

class AbstractTeleopNode(Node):
    def __init__(self, name='abstract_teleop_node'):
        super().__init__(name)
        self.name = name
        self.declare_parameter('host', None)
        self.declare_parameter('port', None)
        self.declare_parameter('endpoint', None)

        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.endpoint = self.get_parameter('endpoint').get_parameter_value().string_value

        if not all([self.host, self.port, self.endpoint]):
            self.get_logger().error(f"[{self.name}] Missing parameters: host, port, endpoint")
            return

        self.websocket_url = f"ws://{self.host}:{self.port}{self.endpoint}"
        self.get_logger().info(f"[{self.name}] Connecting to WebSocket at {self.websocket_url}")

        self.websocket = None
        self.mime = None
        self.message = []
        self.pass_mime = False
        self.fps = 0

        self.network_publisher = self.create_publisher(
            String,
            f"/network_status",
            10
        )

    @abstractmethod
    async def on_received(self, message):
        """
        This method should be implemented by subclasses to handle received messages.
        :param message: The message received from the WebSocket.
        """
        raise NotImplementedError("Subclasses must implement this method.")

    async def set_mime(self, mime):
        """
        Sets the MIME type for the WebSocket connection.
        :param mime: The MIME type to set.
        """

        self.mime = mime
        self.get_logger().info(f"[{self.name}] MIME type set to: {self.mime}")
        try:
            if self.websocket is None:
                self.get_logger().error(f"[{self.name}] WebSocket is not connected")
                return
            await self.websocket.send(self.mime)
            self.mime_sent = True
        except Exception as e:
            self.get_logger().error(f"[{self.name}] Error sending MIME type: {e}")

    async def run(self):
        while True:
            try:
                async with websockets.connect(self.websocket_url, ping_timeout=None) as self.websocket:
                    t1 = asyncio.create_task(self.send())
                    t2 = asyncio.create_task(self.recv())
                    await asyncio.gather(t1, t2)
            except Exception as e:
                self.get_logger().error(f"[{self.name}] WebSocket connection error: {e}")
                await asyncio.sleep(1)

        
    async def recv(self):
        """
        Connects to the WebSocket and listens for messages.
        """
        self.get_logger().info(f"[{self.name}] Connected to WebSocket")
        while True:
            message = await self.websocket.recv()
            # self.get_logger().info(f"[{self.name}] Received message: {message}")
            await self.on_received(message)
            await asyncio.sleep(0.01)
    
    async def send(self):
        """
        Sends the compressed point cloud data to the WebSocket.
        """
        while True:
            if self.pass_mime:
                break
            if self.mime is None:
                await asyncio.sleep(0.1)
                continue
            await self.websocket.send(self.mime)
            break
        while True:
            if self.message:
                await self.websocket.send(self.message.pop(0))
                self.fps += 1
            await asyncio.sleep(0.01)
