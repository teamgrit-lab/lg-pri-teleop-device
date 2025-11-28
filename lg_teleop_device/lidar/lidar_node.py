import rclpy
from rclpy.node import Node
from lg_teleop_device.abstract_teleop_node import AbstractTeleopNode
import DracoPy
import numpy as np
import asyncio, time, json
import threading
from collections import deque
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformListener, Buffer
import tf2_sensor_msgs
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped


class LidarNode(AbstractTeleopNode):
    def __init__(self, name='lidar_node'):
        super().__init__(name)
        self.name = name
        self.declare_parameter('pointcloud_topic1', '')
#        self.declare_parameter('pointcloud_topic2', '')
#        self.declare_parameter('pointcloud_topic3', '')

        self.pointcloud_topic1 = self.get_parameter('pointcloud_topic1').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            PointCloud2,
            self.pointcloud_topic1,
            self.pointcloud1_callback,
            10
        )
        
        self.read_timer = None
        self.processing_timer = None
        self.encoding_timer = None
        self.read_time_diff = 0.0
        self.processing_time_diff = 0.0
        self.encoding_time_diff = 0.0

        self.network_time = 0.0

        self.mime = "lidar/draco"
        self._message_queue = deque(maxlen=120)
        self._send_idle_sleep = 0.002

    def point_cloud_callback(self, cloud1):

        pts, colors = self.extract_points(cloud1, 3.0)

        if pts.size == 0 or colors.size == 0:
            return b''
        if self.processing_timer is not None:
            now = time.time()
            self.processing_time_diff = now - self.processing_timer
            self.processing_timer = None
        if self.encoding_timer is None:
            self.encoding_timer = time.time()
        compressed = DracoPy.encode(
            pts,
            colors=colors,
            faces=None,
            quantization_bits=10,
            compression_level=0
        )
        if self.encoding_timer is not None:
            now = time.time()
            self.encoding_time_diff = now - self.encoding_timer
            self.encoding_timer = None
        json_msg = {
            "name": self.name,
            "processingTimes_ms": {
                "sensorRead": self.read_time_diff * 1000,
                "processing": self.processing_time_diff * 1000,
                "encoding": self.encoding_time_diff * 1000,
            },
            "fps": self.fps
        }
        if time.time() - self.network_time > 1.0:
            self.network_time = time.time()
            json_str = json.dumps(json_msg)
            # self.network_publisher.publish(String(data=json_str))
            self.fps = 0
        self._message_queue.append(compressed)
        if self.read_timer is None:
            self.read_timer = time.time()
        pass

    def pointcloud1_callback(self, cloud_msg: PointCloud2):
        if self.read_timer is not None:
            now = time.time()
            self.read_time_diff = now - self.read_timer
            self.read_timer = None
            
        if self.processing_timer is None:
            self.processing_timer = time.time()
        transform = TransformStamped()
        try:
            transform = self.tf_buffer.lookup_transform('base_link', cloud_msg.header.frame_id, rclpy.time.Time())
#            transform = self.tf_buffer.lookup_transform('base_link', "camera_head_link", rclpy.time.Time())
        except Exception as e:
            self.get_logger().error(f"Failed to get transform: {e}")
            return
        transformed_msg = tf2_sensor_msgs.do_transform_cloud(cloud_msg, transform)
        self.point_cloud_callback(transformed_msg)
     
    async def on_received(self, message):
        print(f"[{self.name}] Received message: {message}")

    async def send(self):
        while True:
            if self.pass_mime:
                break
            if self.mime is None:
                await asyncio.sleep(0.1)
                continue
            await self.websocket.send(self.mime)
            break
        while True:
            if not self._message_queue:
                await asyncio.sleep(self._send_idle_sleep)
                continue
            payload = self._message_queue.popleft()
            await self.websocket.send(payload)
            self.fps += 1

    def extract_points(self, cloud_msg, max_distance):
        num_points = cloud_msg.width * cloud_msg.height
        if num_points == 0:
            return np.empty((0, 3), dtype=np.float32), np.empty((0, 3), dtype=np.uint8)

        point_step = cloud_msg.point_step
        fields = cloud_msg.fields
        data = cloud_msg.data

        raw = np.frombuffer(data, dtype=np.uint8, count=num_points * point_step)
        raw = raw.reshape(num_points, point_step)

        ofs = {f.name: f.offset for f in fields}
        if not all(key in ofs for key in ('x', 'y', 'z')):
            return np.empty((0, 3), dtype=np.float32), np.empty((0, 3), dtype=np.uint8)

        x = raw[:, ofs['x']:ofs['x']+4].view(np.float32).ravel()
        y = raw[:, ofs['y']:ofs['y']+4].view(np.float32).ravel()
        z = raw[:, ofs['z']:ofs['z']+4].view(np.float32).ravel()

        dist2 = x * x + y * y + z * z
        max_distance_sq = max_distance * max_distance
        mask = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & (dist2 < max_distance_sq)

        if not np.any(mask):
            return np.empty((0, 3), dtype=np.float32), np.empty((0, 3), dtype=np.uint8)

        x = x[mask]
        y = y[mask]
        z = z[mask]
        points = np.stack((x, y, z), axis=1).astype(np.float32, copy=False)

        if 'rgb' in ofs:
            rgb = raw[:, ofs['rgb']:ofs['rgb']+4].view(np.uint32).ravel()
            rgb = rgb[mask]
            colors = np.stack(
                ((rgb >> 16) & 0xFF,
                 (rgb >> 8) & 0xFF,
                 rgb & 0xFF),
                axis=1
            ).astype(np.uint8, copy=False)
        else:
            colors = np.zeros((points.shape[0], 3), dtype=np.uint8)

        return points, colors


def main(args=None):
    rclpy.init(args=args)

    temp_node = Node('temp_node')
    temp_node.declare_parameter('name', 'lidar_node')
    name = temp_node.get_parameter('name').get_parameter_value().string_value
    temp_node.destroy_node()

    node = LidarNode(name=name)

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
