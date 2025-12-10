import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
import asyncio, json
import threading, time
from lg_teleop_device.abstract_teleop_node import AbstractTeleopNode
from sensor_msgs.msg import JointState

class JointStatesSubscriptionNode(AbstractTeleopNode):
    def __init__(self, name='joint_states_node'):
        super().__init__(name)
        self.declare_parameter('topic', None)

        self.topic_name = self.get_parameter('topic').get_parameter_value().string_value

        if not self.topic_name:
            self.get_logger().error("No topic name provided via --ros-args.")
            return

        self.publisher = self.create_publisher(JointState, self.topic_name, 10)

        self.mime = f"test/json"
        self.create_timer(10, self.timer_callback)

    def timer_callback(self):
        self.message.append(json.dumps("ping").encode('utf-8'))

    async def on_received(self, message):
        """
        This method is called when a message is received from the WebSocket.
        :param message: The message received from the WebSocket.
        """
        msg = JointState()
        try:
            data = json.loads(message)
            for i in range(len(data['name'])):
                msg.name.append(data['name'][i])
                msg.position.append(data['position'][i])
                
            self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f"[{self.name}] Failed to deserialize message: {e}")
        
def main(args=None):
    rclpy.init(args=args)

    temp_node = Node('temp_node')
    temp_node.declare_parameter('name', 'joint_states_node')
    name = temp_node.get_parameter('name').get_parameter_value().string_value
    temp_node.destroy_node()

    node = JointStatesSubscriptionNode(name=name)

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
