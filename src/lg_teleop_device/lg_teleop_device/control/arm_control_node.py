import rclpy
from rclpy.node import Node
from lg_teleop_device.abstract_teleop_node import AbstractTeleopNode
import threading
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import asyncio, json, math, time
from std_msgs.msg import String
from grp_control_msg.srv import SingleInt, Void


class ArmControlNode(AbstractTeleopNode):
    def __init__(self, name='arm_node'):
        super().__init__(name)
        self.name = name
        self.declare_parameter('tf_target_frame', 'wrist3_link')
        self.tf_target_frame = self.get_parameter('tf_target_frame').get_parameter_value().string_value
        self.pose_topic = f"/{self.name}_pose"
        self.singleint = 0
        self.gripper_state = 99
        if "left" in self.name:
            self.singleint = 2
        else:
            self.singleint = 1
        self.sync_topic = f"/{self.name}_sync_trigger"
        self.ai_mode_topic = f"/{self.name}_ai_mode"
        self.gripper_topic = f"/{self.name}_gripper"

        self.pose_buffer = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(
            PoseStamped,
            self.pose_topic,
            10
        )
        self.sync_publisher = self.create_publisher(
            Bool,
            self.sync_topic,
            10
        )
        self.ai_mode_publisher = self.create_publisher(
            Bool,
            self.ai_mode_topic,
            10
        )
        self.gripper_publisher = self.create_publisher(
            Bool,
            self.gripper_topic,
            10
        )

        self.gripper_select_cli = self.create_client(SingleInt, "/modbus_slave_change")
        while not self.gripper_select_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.gripper_open_cli = self.create_client(Void, "/grp_open")
        while not self.gripper_open_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.gripper_close_cli = self.create_client(Void, "/grp_close")
        while not self.gripper_close_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.pass_mime = True
        self.gripper_trigger = False
        self.sync_trigger = False
        self.ai_mode_trigger = False

        self.create_timer(0.2, self.publish_timer_callback)
        self.create_timer(0.02, self.tf_callback)
        self.create_timer(10.0, self.timer_callback)

        self.timer = None

    async def on_received(self, message):
        """
        Handle received messages and publish them to the control topic.
        :param message: The message received from the WebSocket.
        """
        if self.timer is None:
            self.timer = time.time()
        try:
            json_data = json.loads(message)
            current_pose = PoseStamped()
            if 'position' in json_data:
                current_pose.header.stamp = self.get_clock().now().to_msg()
                current_pose.header.frame_id = 'base_link'
                current_pose.pose.position.x = -float(json_data['position']['z'])
                current_pose.pose.position.y = -float(json_data['position']['x'])
                current_pose.pose.position.z = float(json_data['position']['y']) + 1.1
                current_pose.pose.orientation.x = -float(json_data['orientation']['z'])
                current_pose.pose.orientation.y = -float(json_data['orientation']['x'])
                current_pose.pose.orientation.z = float(json_data['orientation']['y'])
                current_pose.pose.orientation.w = float(json_data['orientation']['w'])

                self.pose_buffer.append(current_pose)
                if len(self.pose_buffer) > 12:
                    self.pose_buffer.pop(0)
                
                average_pose = PoseStamped()
                sum_x = 0.0
                sum_y = 0.0
                sum_z = 0.0
                sum_exp = 0.0

                count = 0
                for pose in self.pose_buffer:
                    sum_exp += math.exp((count/12.0) - 1.0)
                    sum_x += pose.pose.position.x * math.exp((count/12.0) - 1.0)
                    sum_y += pose.pose.position.y * math.exp((count/12.0) - 1.0)
                    sum_z += pose.pose.position.z * math.exp((count/12.0) - 1.0)
                    count += 1
                
                average_pose.pose.position.x = sum_x / sum_exp
                average_pose.pose.position.y = sum_y / sum_exp
                average_pose.pose.position.z = sum_z / sum_exp
                average_pose.header.stamp = self.get_clock().now().to_msg()
                average_pose.header.frame_id = 'base_link'
                average_pose.pose.orientation = current_pose.pose.orientation

                self.publisher.publish(average_pose)
                if self.timer is not None:
                    now = time.time()
                    time_diff = now - self.timer
                    self.timer = None
                    json_msg = {
                        "name": self.name,
                        "processingTimes_ms": time_diff * 1000,
                    }
                    json_str = json.dumps(json_msg)
                    self.network_publisher.publish(String(data=json_str))
            
            if 'gripper' in json_data:
                if self.gripper_state == json_data['gripper']:
                    pass
                else:
                    self.gripper_state = json_data['gripper']
                    req = SingleInt.Request()
                    req.value = self.singleint
                    future = self.gripper_select_cli.call_async(req)
                    req2 = Void.Request()
                    if json_data['gripper']:
                        future2 = self.gripper_open_cli.call_async(req2)
                    else:
                        future2 = self.gripper_close_cli.call_async(req2)
#                    self.gripper_trigger = bool(json_data['gripper'])
            if 'sync' in json_data:
                self.sync_trigger = bool(json_data['sync'])
            if 'ai_mode' in json_data:
                self.ai_mode_trigger = bool(json_data['ai_mode'])

        except Exception as e:
            self.get_logger().error(f"[{self.name}] Error processing message: {e}")

    def publish_timer_callback(self):
        self.sync_publisher.publish(Bool(data=self.sync_trigger))
        self.ai_mode_publisher.publish(Bool(data=self.ai_mode_trigger))
        self.gripper_publisher.publish(Bool(data=self.gripper_trigger))

    def timer_callback(self):
        """
        Timer callback to periodically log a message.
        """
        self.message.append(b"ping")

    def tf_callback(self):
        transform_stamped = TransformStamped()
        # 3sec find
        try:
            transform_stamped = self.tf_buffer.lookup_transform('base_link', self.tf_target_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().error(f"Failed to get transform for {self.tf_target_frame}: {e}")
            return
        if not transform_stamped:
            self.get_logger().error(f"Failed to get transform for {self.tf_target_frame}")
            return
        json_data = {
            "type" : "type_pose",
            "handedness" : self.name,
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
            "gripper": self.gripper_trigger,
        }
        self.message.append(json.dumps(json_data).encode('utf-8'))

def main(args=None):
    rclpy.init(args=args)

    temp_node = Node('temp_node')
    temp_node.declare_parameter('name', 'arm_node')
    name = temp_node.get_parameter('name').get_parameter_value().string_value
    temp_node.destroy_node()

    node = ArmControlNode(name=name)

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
