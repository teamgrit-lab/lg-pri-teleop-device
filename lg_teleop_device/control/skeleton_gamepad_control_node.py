import rclpy
from rclpy.node import Node
from lg_teleop_device.abstract_teleop_node import AbstractTeleopNode
import threading, asyncio, json, time
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String, Float32
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped


class SkeletonGamepadControlNode(AbstractTeleopNode):
    def __init__(self, name='skeleton_gamepad_control_node'):
        super().__init__(name)
        # Parameters
        self.declare_parameter('gamepad_control_topic', '/gamepad_control')
        self.declare_parameter('tf_target_frame', 'wrist3_link')
        self.declare_parameter('gripper_button_index', 0)
        self.declare_parameter('handedness', 'right')  # 'left' or 'right' to select modbus index

        self.control_topic = self.get_parameter('gamepad_control_topic').get_parameter_value().string_value
        self.tf_target_frame = self.get_parameter('tf_target_frame').get_parameter_value().string_value
        self.gripper_button_index = self.get_parameter('gripper_button_index').get_parameter_value().integer_value
        self.handedness = self.get_parameter('handedness').get_parameter_value().string_value

        # Normalize handedness to topic suffix (default right)
        self.handedness_suffix = 'left' if 'left' in self.handedness.lower() else 'right'

        # ROS interfaces
        self.publisher = self.create_publisher(Joy, self.control_topic, 10)
        self.gripper_topic = f"/{self.name}_gripper"
        self.gripper_publisher = self.create_publisher(Bool, self.gripper_topic, 10)

        # TF interfaces
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Gripper topic publishers
        self.gripper_open_topic = f"/get/grp/open/{self.handedness_suffix}"
        self.gripper_close_topic = f"/get/grp/close/{self.handedness_suffix}"
        self.gripper_open_publisher = self.create_publisher(Float32, self.gripper_open_topic, 10)
        self.gripper_close_publisher = self.create_publisher(Float32, self.gripper_close_topic, 10)
        self.gripper_command_value = 30.0

        # States
        self.pass_mime = True
        self.gripper_state = 99  # unknown initial
        self.last_button_state = 0
        self.desired_gripper = False
        self.processing_timer = None
        self.network_time = 0.0

        # Timers
        self.create_timer(1.0, self.timer_callback)
        self.create_timer(0.02, self.tf_callback)
        self.create_timer(0.2, self.publish_timer_callback)

    async def on_received(self, message):
        try:
            data = json.loads(message)
            # Forward gamepad values to topic
            if 'axes' in data:
                msg = Joy()
                msg.axes = list(map(float, data['axes']))
                msg.buttons = list(map(int, data.get('buttons', [])))
                self.publisher.publish(msg)

            # Handle gripper on button rising edge
            buttons = data.get('buttons', [])
            current = 1 if (len(buttons) > self.gripper_button_index and buttons[self.gripper_button_index]) else 0
            if current == 1 and self.last_button_state == 0:
                # toggle gripper desired state
                self.desired_gripper = not self.desired_gripper
                await self._trigger_gripper(self.desired_gripper)
            self.last_button_state = current

        except Exception as e:
            self.get_logger().error(f"[{self.name}] Error processing message: {e}")

    async def _trigger_gripper(self, open_gripper: bool):
        try:
            msg = Float32()
            msg.data = self.gripper_command_value
            if open_gripper:
                self.gripper_open_publisher.publish(msg)
            else:
                self.gripper_close_publisher.publish(msg)
            self.gripper_state = 1 if open_gripper else 0
        except Exception as e:
            self.get_logger().error(f"[{self.name}] Failed to trigger gripper: {e}")

    def publish_timer_callback(self):
        self.gripper_publisher.publish(Bool(data=self.desired_gripper))

    def timer_callback(self):
        self.message.append(b"ping")

    def tf_callback(self):
        transform_stamped = TransformStamped()
        try:
            transform_stamped = self.tf_buffer.lookup_transform('base_link', self.tf_target_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().error(f"Failed to get transform for {self.tf_target_frame}: {e}")
            return
        if not transform_stamped:
            self.get_logger().error(f"Failed to get transform for {self.tf_target_frame}")
            return

        json_data = {
            "type": "type_pose",
            "handedness": self.handedness,
            "position": {
                "x": -transform_stamped.transform.translation.y,
                "y": transform_stamped.transform.translation.z - 1.1,
                "z": -transform_stamped.transform.translation.x
            },
            "orientation": {
                "x": -transform_stamped.transform.rotation.y,
                "y": transform_stamped.transform.rotation.z,
                "z": -transform_stamped.transform.rotation.x,
                "w": transform_stamped.transform.rotation.w
            },
            "gripper": self.desired_gripper,
        }
        self.message.append(json.dumps(json_data).encode('utf-8'))


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



