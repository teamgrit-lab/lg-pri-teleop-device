import rclpy
from rclpy.node import Node
import asyncio, json, threading
from lg_teleop_device.abstract_teleop_node import AbstractTeleopNode
from sensor_msgs.msg import Imu, MagneticField


class ImuSubscriptionNode(AbstractTeleopNode):
    def __init__(self, name='imu_subscription_node'):
        super().__init__(name)
        self.declare_parameter('topic', None)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('mag_topic', None)

        self.topic_name = self.get_parameter('topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.mag_topic = self.get_parameter('mag_topic').get_parameter_value().string_value

        if not self.topic_name:
            self.get_logger().error("No topic name provided via --ros-args.")
            return

        self.publisher = self.create_publisher(Imu, self.topic_name, 10)
        self.mag_publisher = None
        if self.mag_topic:
            self.mag_publisher = self.create_publisher(MagneticField, self.mag_topic, 10)

        self.mime = f"test/json"
        self.create_timer(10, self.timer_callback)

    def timer_callback(self):
        self.message.append(json.dumps("ping").encode('utf-8'))

    def _fill_vector3(self, target, src):
        if src is None:
            return
        if 'x' in src:
            target.x = float(src['x'])
        if 'y' in src:
            target.y = float(src['y'])
        if 'z' in src:
            target.z = float(src['z'])

    def _fill_quaternion(self, target, src):
        if src is None:
            return
        if 'x' in src:
            target.x = float(src['x'])
        if 'y' in src:
            target.y = float(src['y'])
        if 'z' in src:
            target.z = float(src['z'])
        if 'w' in src:
            target.w = float(src['w'])

    def _fill_covariance(self, arr, src):
        if not src:
            return
        try:
            for i in range(min(9, len(src))):
                arr[i] = float(src[i])
        except Exception:
            pass

    async def on_received(self, message):
        msg = Imu()
        try:
            data = json.loads(message)

            msg.header.stamp = self.get_clock().now().to_msg()
            if isinstance(data, dict) and 'frame_id' in data and data['frame_id']:
                msg.header.frame_id = str(data['frame_id'])
            else:
                msg.header.frame_id = self.frame_id

            self._fill_quaternion(msg.orientation, data.get('orientation'))
            self._fill_vector3(msg.angular_velocity, data.get('angular_velocity'))
            self._fill_vector3(msg.linear_acceleration, data.get('linear_acceleration'))

            self._fill_covariance(msg.orientation_covariance, data.get('orientation_covariance'))
            self._fill_covariance(msg.angular_velocity_covariance, data.get('angular_velocity_covariance'))
            self._fill_covariance(msg.linear_acceleration_covariance, data.get('linear_acceleration_covariance'))

            self.publisher.publish(msg)

            # Optional MagneticField publishing when provided
            if self.mag_publisher and ('magnetic_field' in data or 'magnetic_field_covariance' in data):
                mag_msg = MagneticField()
                mag_msg.header = msg.header
                self._fill_vector3(mag_msg.magnetic_field, data.get('magnetic_field'))
                self._fill_covariance(mag_msg.magnetic_field_covariance, data.get('magnetic_field_covariance'))
                self.mag_publisher.publish(mag_msg)
        except Exception as e:
            self.get_logger().error(f"[{self.name}] Failed to deserialize IMU message: {e}")


def main(args=None):
    rclpy.init(args=args)

    temp_node = Node('temp_node')
    temp_node.declare_parameter('name', 'imu_subscription_node')
    name = temp_node.get_parameter('name').get_parameter_value().string_value
    temp_node.destroy_node()

    node = ImuSubscriptionNode(name=name)

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


