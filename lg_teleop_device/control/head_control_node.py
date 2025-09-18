import rclpy
from rclpy.node import Node
from lg_teleop_device.abstract_teleop_node import AbstractTeleopNode
import threading
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
import asyncio, json, math
#from tf_transformations import euler_from_quaternion


class HeadControlNode(AbstractTeleopNode):
    def __init__(self, name='head_control_node'):
        super().__init__(name)

        self.pitch_topic = f"/{self.name}/yaw"
        self.yaw_topic = f"/{self.name}/pitch"
        self.pose_topic = f"/{self.name}/pose"

        self.pitch_publisher = self.create_publisher(
            Float32,
            self.pitch_topic,
            10
        )
        self.yaw_publisher = self.create_publisher(
            Float32,
            self.yaw_topic,
            10
        )
        # self.pose_publisher = self.create_publisher(
        #     PoseStamped,
        #     self.pose_topic,
        #     10
        # )

        self.create_timer(10.0, self.timer_callback)
        self.pass_mime = True

    async def on_received(self, message):
        """
        Handle received messages and publish them to the control topic.
        :param message: The message received from the WebSocket.
        """
        try:
            json_data = json.loads(message)
            print(message)
            if 'orientation' not in json_data:
                self.get_logger().error(f"[{self.name}] Invalid message format: {message}")
                return
            head_yaw_msg = Float32()

#            q = [
            roll = -float(json_data['orientation']['z'])
            pitch = -float(json_data['orientation']['x'])
            yaw = float(json_data['orientation']['y'])
#                float(json_data['orientation']['w']),
#            ]

#            roll, pitch, yaw = euler_from_quaternion(q)
            yaw_data = math.pi + yaw
            if yaw_data > math.pi + (math.pi / 2):
                yaw_data = float(math.pi + (math.pi / 2))
            elif yaw_data < math.pi / 2:
                yaw_data = float(math.pi / 2)
#            head_yaw_msg.data = math.pi - yaw
            head_yaw_msg.data = yaw_data
            self.yaw_publisher.publish(head_yaw_msg)


            head_pitch_msg = Float32()
            data = float(pitch + math.pi / 4)
            if data > 1.4:
                data = float(1.4)
            elif data < 0.0:
                data = float(0.0)
            head_pitch_msg.data = data
            self.pitch_publisher.publish(head_pitch_msg)

            # pose_msg = PoseStamped()
            # pose_msg.header.stamp = self.get_clock().now().to_msg()
            # pose_msg.header.frame_id = "base_link"
            # pose_msg.pose.position.x = -float(json_data['position']['z'])
            # pose_msg.pose.position.y = -float(json_data['position']['x'])
            # pose_msg.pose.position.z = float(json_data['position']['y']) + 1.1
            # pose_msg.pose.orientation.x = -float(json_data['orientation']['z'])
            # pose_msg.pose.orientation.y = -float(json_data['orientation']['x'])
            # pose_msg.pose.orientation.z = float(json_data['orientation']['y'])
            # pose_msg.pose.orientation.w = float(json_data['orientation']['w'])

            # self.pose_publisher.publish(pose_msg)
            
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
    temp_node.declare_parameter('name', 'head_control_node')
    name = temp_node.get_parameter('name').get_parameter_value().string_value
    temp_node.destroy_node()

    node = HeadControlNode(name=name)

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
