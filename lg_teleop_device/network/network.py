import rclpy
from rclpy.node import Node
import asyncio
import threading, json
from lg_teleop_device.abstract_teleop_node import AbstractTeleopNode
from std_msgs.msg import String
import time


class NetworkNode(AbstractTeleopNode):
    def __init__(self, name='network_node'):
        super().__init__(name)
        self.create_timer(1, self.timer_callback)
        suber = self.create_subscription(String, "/network_status", self.network_callback, 10)
        self.pass_mime = True

    async def on_received(self, message):
        """
        Handle received messages and publish them to the control topic.
        :param message: The message received from the WebSocket.
        """
        print(f"[{self.name}] Received message: {message}")

    def network_callback(self, msg):
        try:
            json_msg = bytes(msg.data, 'utf-8')
            self.message.append(json_msg)
        except Exception as e:
            self.get_logger().error(f"[{self.name}] Failed to process network message: {e}")

    def timer_callback(self):
        """
        Timer callback to periodically log a message.
        """
        self.message.append(b"ping")

def main(args=None):
    rclpy.init(args=args)

    temp_node = Node('temp_node')
    temp_node.declare_parameter('name', 'network_node')
    name = temp_node.get_parameter('name').get_parameter_value().string_value
    temp_node.destroy_node()

    node = NetworkNode(name=name)

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
