import rclpy
from rclpy.node import Node
import asyncio
import threading, json
from lg_teleop_device.abstract_teleop_node import AbstractTeleopNode
from sensor_msgs.msg import Joy
import time


class GamepadControlNode(AbstractTeleopNode):
    def __init__(self, name='gamepad_control_node'):
        super().__init__(name)
        self.declare_parameter('gamepad_control_topic', '/gamepad_control')
        self.control_topic = self.get_parameter('gamepad_control_topic').get_parameter_value().string_value
        self.publisher = self.create_publisher(
            Joy,
            self.control_topic,
            10
        )
        self.create_timer(1, self.timer_callback)
        self.pass_mime = True

    async def on_received(self, message):
        """
        Handle received messages and publish them to the control topic.
        :param message: The message received from the WebSocket.
        """
        try:
            gamepad_msg = Joy()
            data = json.loads(message)
#            self.get_logger().info(f"{type(data['axes'][0])}")
            gamepad_msg.axes = list(map(float, data['axes']))
            gamepad_msg.buttons = list(map(float, data.get('buttons', [])))
            self.get_logger().info(f"[{self.name}] Publishing Gamepad message: {gamepad_msg}")
            self.publisher.publish(gamepad_msg)
        except Exception as e:
            self.get_logger().error(f"[{self.name}] Error processing message: {e}")

    def timer_callback(self):
        """
        Timer callback to periodically log a message.
        """
        self.message.append(b"ping")

def main(args=None):
    rclpy.init(args=args)

    temp_node = Node('temp_node')
    temp_node.declare_parameter('name', 'gamepad_control_node')
    name = temp_node.get_parameter('name').get_parameter_value().string_value
    temp_node.destroy_node()

    node = GamepadControlNode(name=name)

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
