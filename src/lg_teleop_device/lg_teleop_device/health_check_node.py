import rclpy
from rclpy.node import Node
from lg_teleop_device.abstract_teleop_node import AbstractTeleopNode
import asyncio
import threading
import json
import requests

class HealthCheckNode(AbstractTeleopNode):
    def __init__(self, name='health_check_node'):
        super().__init__(name)
        self.declare_parameter('token', None)
        self.token = self.get_parameter('token').get_parameter_value().string_value
        self.pass_mime = True

        if not self.token:
            self.get_logger().error("No token provided via --ros-args.")
            return

    async def on_received(self, message):
        """
        This method is called when a message is received from the WebSocket.
        :param message: The message received from the WebSocket.
        """
        json_data = json.loads(message)
        if 'type' in json_data:
            if json_data['type'] == 'HEALTHCHECK':
                response = {
                    'type': 'HEARTBEAT',
                    'token': self.token
                }
                self.message.append(json.dumps(response).encode('utf-8'))
                # self.get_logger().info(f"[{self.name}] Sent health response: {response}")
            elif json_data['type'] == 'TASK_REGISTERED':
                self.get_logger().info(f"[{self.name}] Task registered: {message}")

def main(args=None):
    rclpy.init(args=args)

    temp_node = Node('temp_node')
    temp_node.declare_parameter('name', 'health_check_node')
    name = temp_node.get_parameter('name').get_parameter_value().string_value
    temp_node.destroy_node()

    node = HealthCheckNode(name=name)

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