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
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            self.pose_topic,
            10
        )

        self.create_timer(10.0, self.timer_callback)
        self.pass_mime = True
        
    def calculate_servo_angles(self, pose_stamped):
        """
        PoseStamped(Orientation) -> Servo Value (Float32)
        - Pitch: 1.2(Down, -45deg) ~ 0.6(Up, +45deg)
        - Yaw: 3.14 Center, +-60deg
        """
        q = pose_stamped.pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w

        norm = math.sqrt(x*x + y*y + z*z + w*w)
        
        if norm == 0:
            return 3.14, 0.9
        
        x /= norm
        y /= norm
        z /= norm
        w /= norm

        sinp = 2.0 * (w * y - z * x)
        
        if abs(sinp) >= 1:
            pitch_rad = math.copysign(math.pi / 2, sinp)
        else:
            pitch_rad = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)

        YAW_CENTER = 3.14
        YAW_LIMIT = math.radians(60)
        
        target_yaw = YAW_CENTER + yaw_rad
        target_yaw = max(YAW_CENTER - YAW_LIMIT, min(target_yaw, YAW_CENTER + YAW_LIMIT))

        SLOPE = 0.382
        INTERCEPT = 0.95
        
        target_pitch = (SLOPE * pitch_rad) + INTERCEPT
        target_pitch = max(0.6, min(target_pitch, 1.2))
        return target_yaw, target_pitch

    async def on_received(self, message):
        """
        Handle received messages and publish them to the control topic.
        :param message: The message received from the WebSocket.
        """
        try:
            json_data = json.loads(message)
            self.get_logger().info(f"[{self.name}] Received message: {message}")
            if 'orientation' not in json_data:
                self.get_logger().error(f"[{self.name}] Invalid message format: {message}")
                return

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "base_link"
            pose_msg.pose.position.x = -float(json_data['position']['z'])
            pose_msg.pose.position.y = -float(json_data['position']['x'])
            pose_msg.pose.position.z = float(json_data['position']['y']) + 1.1
            pose_msg.pose.orientation.x = -float(json_data['orientation']['z'])
            pose_msg.pose.orientation.y = -float(json_data['orientation']['x'])
            pose_msg.pose.orientation.z = float(json_data['orientation']['y'])
            pose_msg.pose.orientation.w = float(json_data['orientation']['w'])

            self.pose_publisher.publish(pose_msg)

            yaw_value, pitch_value = self.calculate_servo_angles(pose_msg)
            head_yaw_msg = Float32()
            head_yaw_msg.data = yaw_value
            self.yaw_publisher.publish(head_yaw_msg)
            head_pitch_msg = Float32()
            head_pitch_msg.data = pitch_value
            self.pitch_publisher.publish(head_pitch_msg)
            
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
