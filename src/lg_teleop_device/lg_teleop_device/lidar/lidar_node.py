import rclpy
from rclpy.node import Node
from lg_teleop_device.abstract_teleop_node import AbstractTeleopNode
import DracoPy
import numpy as np
import asyncio, time, json
import threading
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

    def point_cloud_callback(self, cloud1):

        point_data1, color_data1, valid_points1 = self.extract_points(cloud1, 3)
        point_data1 = point_data1[:valid_points1 * 3]
        color_data1 = color_data1[:valid_points1 * 3]
        point_data = point_data1
        color_data = color_data1


        pts = np.array(point_data, dtype=np.float32).reshape(-1, 3)
        colors = np.array(color_data, dtype=np.uint8).reshape(-1, 3)
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
        self.message.append(compressed)
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

    def extract_points(self, cloud_msg, skip_length_):
        # 1) 준비
        num_points = cloud_msg.width * cloud_msg.height
        point_step = cloud_msg.point_step
        fields = cloud_msg.fields
        data = cloud_msg.data  # bytes

        # 2) raw buffer → (N, point_step) uint8 배열
        raw = np.frombuffer(data, dtype=np.uint8).reshape(num_points, point_step)

        # 3) 각 필드 오프셋
        ofs = {f.name: f.offset for f in fields}

        # 4) x,y,z 한 번에 읽기 (각각 shape (N,))
        x = raw[:, ofs['x']:ofs['x']+4].view(np.float32).ravel()
        y = raw[:, ofs['y']:ofs['y']+4].view(np.float32).ravel()
        z = raw[:, ofs['z']:ofs['z']+4].view(np.float32).ravel()

        # 5) NaN & 거리 필터 (제곱근 제거)
        dist2 = x*x + y*y + z*z
        mask = (~np.isnan(x)) & (~np.isnan(y)) & (~np.isnan(z)) & (dist2 < skip_length_**2)

        x = x[mask];  y = y[mask];  z = z[mask]

        # 6) RGB가 있으면 한 번에 처리
        if 'rgb' in ofs:
            rgb_f = raw[:, ofs['rgb']:ofs['rgb']+4].view(np.float32).ravel()
            rgb_u = rgb_f.view(np.uint32)[mask]
            # 비트 연산
            r = ((rgb_u >> 16) & 0xFF).astype(np.uint8)
            g = ((rgb_u >>  8) & 0xFF).astype(np.uint8)
            b = ( rgb_u        & 0xFF).astype(np.uint8)
        else:
            # default black
            n = x.shape[0]
            r = g = b = np.zeros(n, dtype=np.uint8)

        # 7) 결과로 쓸 리스트 생성 (필요하면 NumPy 형태 그대로도 사용 가능)
        point_data = np.stack([x, y, z], axis=1).ravel().tolist()
        color_data = np.stack([r, g, b], axis=1).ravel().tolist()
        valid_points = x.shape[0]

        return point_data, color_data, valid_points


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
