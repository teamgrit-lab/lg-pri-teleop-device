import rclpy
from rclpy.node import Node
import asyncio
import threading, json
from lg_teleop_device.abstract_teleop_node import AbstractTeleopNode
from std_msgs.msg import Int32
import time


class WaistControlNode(AbstractTeleopNode):
    def __init__(self, name='waist_control_node'):
        super().__init__(name)
        self.declare_parameter('waist_control_topic', '/waist_control')
        self.control_topic = self.get_parameter('waist_control_topic').get_parameter_value().string_value
        self.declare_parameter('waist_control_speed_mm_per_sec', 100.0)
        self.speed_mm_per_sec = self.get_parameter('waist_control_speed_mm_per_sec').get_parameter_value().double_value
        self.current_position = 0
        self.triggered = False
        self.time = 0.0
        self.suber = self.create_subscription(
            Int32,
            '/motor_current_position',
            self.feedback_callback,
            10
        )
        self.publisher = self.create_publisher(
            Int32,
            self.control_topic,
            10
        )

        self.create_timer(10, self.timer_callback)
        self.pass_mime = True

    def feedback_callback(self, msg):
        self.current_position = msg.data
        json_msg = {
            "waist": self.current_position

        }
        self.message.append(json.dumps(json_msg).encode('utf-8'))
        
        if not self.triggered:
            self.triggered = True

    async def on_received(self, message):
        """
        Handle received messages and publish them to the control topic.
        :param message: The message received from the WebSocket.
        """
        try:
            if self.triggered:
                if time.time() - self.time < 0.1:
                    return
                self.time = time.time()
                waist_msg = Int32()
                data = json.loads(message)
                axes_3 = -float(data['axes'][2])
                current_position = self.current_position + int(axes_3 * self.speed_mm_per_sec * (0.1))
                self.get_logger().info(f"[{self.name}] axes_3: {axes_3}, current_position: {self.current_position}, speed: {self.speed_mm_per_sec}, current_position: {current_position}")
                waist_msg.data = current_position
                self.publisher.publish(waist_msg)


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

    node = WaistControlNode(name=name)

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
