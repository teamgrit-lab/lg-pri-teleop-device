import rclpy
from rclpy.node import Node
from lg_teleop_device.abstract_teleop_node import AbstractTeleopNode
import threading
from geometry_msgs.msg import Twist
import asyncio, json


class TwistControlNode(AbstractTeleopNode):
    def __init__(self, name='twist_control_node'):
        super().__init__(name)
        self.declare_parameter('topic', '/cmd_vel')
        self.control_topic = self.get_parameter('topic').get_parameter_value().string_value
        self.publisher = self.create_publisher(
            Twist,
            self.control_topic,
            10
        )
        self.create_timer(10.0, self.timer_callback)
        self.pass_mime = True

    async def on_received(self, message):
        """
        Handle received messages and publish them to the control topic.
        :param message: The message received from the WebSocket.
        """
        try:
            json_data = json.loads(message)
            if 'axes' not in json_data or len(json_data['axes']) < 3:
                self.get_logger().error(f"[{self.name}] Invalid message format: {message}")
                return
            twist_msg = Twist()
            twist_msg.linear.x = -float(json_data['axes'][1])
            twist_msg.linear.y = -float(json_data['axes'][0])
            twist_msg.angular.z = -float(json_data['axes'][2])
            self.get_logger().info(f"[{self.name}] Publishing Twist message: {twist_msg}")
            self.publisher.publish(twist_msg)
        except Exception as e:
            self.get_logger().error(f"[{self.name}] Error processing message: {e}")

    def timer_callback(self):
        """
        Timer callback to periodically log a message.
        """
        print(f"[{self.name}] 123")
        self.message.append(b"ping")

def main(args=None):
    rclpy.init(args=args)

    temp_node = Node('temp_node')
    temp_node.declare_parameter('name', 'twist_control_node')
    name = temp_node.get_parameter('name').get_parameter_value().string_value
    temp_node.destroy_node()

    node = TwistControlNode(name=name)

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
