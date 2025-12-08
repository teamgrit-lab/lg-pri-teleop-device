import rclpy
from rclpy.node import Node
from lg_teleop_device.abstract_teleop_node import AbstractTeleopNode
import threading, asyncio, json
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32


class SkeletonGamepadControlNode(AbstractTeleopNode):
    def __init__(self, name='skeleton_gamepad_control_node'):
        super().__init__(name)
        # Parameters
        self.declare_parameter('gamepad_control_topic', '/gamepad_control')
        self.declare_parameter('gripper_button_index', 0)
        self.declare_parameter('handedness', 'right')  # 'left' or 'right' to select modbus index
        self.control_topic = self.get_parameter('gamepad_control_topic').get_parameter_value().string_value
        self.gripper_button_index = self.get_parameter('gripper_button_index').get_parameter_value().integer_value
        self.handedness = self.get_parameter('handedness').get_parameter_value().string_value

        self.singleint = 2 if 'left' in self.handedness.lower() else 1
        self.publisher = self.create_publisher(Joy, self.control_topic, 10)
        self.gripper_init_topic = f'/get/grp/init/{self.handedness.lower()}'
        self.gripper_open_topic = f'/get/grp/open/{self.handedness.lower()}'
        self.gripper_close_topic = f'/get/grp/close/{self.handedness.lower()}'
        self.gripper_command_value = 30.0

        self.gripper_init = self.create_publisher(Float32, self.gripper_init_topic, 10)
        self.gripper_open = self.create_publisher(Float32, self.gripper_open_topic, 10)
        self.gripper_close = self.create_publisher(Float32, self.gripper_close_topic, 10)

        self.pass_mime = True
        self.last_button_state = 0
        self.desired_gripper = False

        self.create_timer(1.0, self.timer_callback)

        self.gripper_init.publish(Float32(data=1.0))

    async def on_received(self, message):
        try:
            data = json.loads(message)
            if 'axes' in data:
                msg = Joy()
                msg.axes = list(map(float, data['axes']))
                msg.buttons = list(map(int, data.get('buttons', [])))
                self.publisher.publish(msg)

            buttons = data.get('buttons', [])
            current = 1 if (len(buttons) > self.gripper_button_index and buttons[self.gripper_button_index]) else 0
            if current == 1 and self.last_button_state == 0:
                self.desired_gripper = not self.desired_gripper
                await self._trigger_gripper(self.desired_gripper)
            self.last_button_state = current

        except Exception as e:
            self.get_logger().error(f"[{self.name}] Error processing message: {e}")

    async def _trigger_gripper(self, open_gripper: bool):
        try:
            msg = Float32()
            if open_gripper:
                msg.data = self.gripper_command_value
                self.gripper_open.publish(msg)
            else:
                msg.data = self.gripper_command_value
                self.gripper_close.publish(msg)
        except Exception as e:
            self.get_logger().error(f"[{self.name}] Error triggering gripper: {e}")

    def timer_callback(self):
        self.message.append(b"ping")


def main(args=None):
    rclpy.init(args=args)

    temp_node = Node('temp_node')
    temp_node.declare_parameter('name', 'skeleton_gamepad_control_node')
    name = temp_node.get_parameter('name').get_parameter_value().string_value
    temp_node.destroy_node()

    node = SkeletonGamepadControlNode(name=name)

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



