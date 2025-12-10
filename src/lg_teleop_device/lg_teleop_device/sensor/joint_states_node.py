import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
import asyncio, json
import threading, time
from lg_teleop_device.abstract_teleop_node import AbstractTeleopNode
from sensor_msgs.msg import JointState

class JointStatesNode(AbstractTeleopNode):
    def __init__(self, name='joint_states_node'):
        super().__init__(name)
        self.declare_parameter('topic', None)

        self.topic_name = self.get_parameter('topic').get_parameter_value().string_value

        if not self.topic_name:
            self.get_logger().error("No topic name provided via --ros-args.")
            return

        self.get_logger().info(f"[{self.name}] Subscribing to topic: {self.topic_name}")

        self.suber = self.create_subscription(
            JointState,
            self.topic_name,
            self.callback,
            10
        )

        self.json_str = {
            'name': [],
            'position': [],
            'isUpdated': False
        }
        self.name_to_index = {}

        self.mime = f"test/json"
        self.create_timer(0.02, self.timer_callback)
        self.ping_timer_ = time.time()

    def timer_callback(self):
        if not self.json_str['isUpdated']:
            if time.time() - self.ping_timer_ > 5:
                self.message.append(b"ping")
                self.ping_timer_ = time.time()
            return
        self.message.append(json.dumps(self.json_str).encode('utf-8'))
        self.ping_timer_ = time.time()
        self.json_str['isUpdated'] = False

    def get_topic_type(self, topic_name):
        topic_info = self.get_topic_names_and_types()
        for name, types in topic_info:
            if name == topic_name:
                return types[0] if types else None
        return None

    def callback(self, msg):
        try:
            # 초기 메시지에서 동적 초기화
            if not self.json_str['name']:
                self.json_str['name'] = list(msg.name)
                self.json_str['position'] = list(msg.position)
                self.json_str['isUpdated'] = True
                self.name_to_index = {name: i for i, name in enumerate(self.json_str['name'])}
                return

            # 이후 메시지는 이름 기준으로 위치값 업데이트/추가
            for i, name in enumerate(msg.name):
                if name in self.name_to_index:
                    idx = self.name_to_index[name]
                    if i < len(msg.position):
                        self.json_str['position'][idx] = msg.position[i]
                else:
                    self.name_to_index[name] = len(self.json_str['name'])
                    self.json_str['name'].append(name)
                    self.json_str['position'].append(msg.position[i] if i < len(msg.position) else 0.0)
            self.json_str['isUpdated'] = True

        except Exception as e:
            self.get_logger().error(f"[{self.name}] Failed to deserialize message: {e}")
    
    async def on_received(self, message):
        """
        This method is called when a message is received from the WebSocket.
        :param message: The message received from the WebSocket.
        """
        # Process the received message here
        self.get_logger().info(f"[{self.name}] Received message: {message}")
        
def main(args=None):
    rclpy.init(args=args)

    temp_node = Node('temp_node')
    temp_node.declare_parameter('name', 'joint_states_node')
    name = temp_node.get_parameter('name').get_parameter_value().string_value
    temp_node.destroy_node()

    node = JointStatesNode(name=name)

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
