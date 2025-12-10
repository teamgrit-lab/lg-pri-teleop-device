import asyncio
import json
import threading
from typing import Any, Optional

import rclpy
from rclpy.node import Node

from lg_teleop_device.abstract_teleop_node import AbstractTeleopNode
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Illuminance, Imu, MagneticField, NavSatFix, NavSatStatus


class MobileSensorSubscriptionNode(AbstractTeleopNode):
    """
    Teleop 서버에서 수신한 모바일 디바이스 센서 JSON을 ROS 토픽으로 재발행하는 노드.
    JSON 예시:
    {
        "imuData": {
            "accelX": 0.0, "accelY": 0.0, "accelZ": 9.8,
            "gyroX": 0.0, "gyroY": 0.0, "gyroZ": 0.0,
            "magX": 30.0, "magY": 2.0, "magZ": -50.0,
            "light": 123.0
        },
        "locationData": {
            "latitude": 37.0, "longitude": 127.0, "altitude": 35.0,
            "speed": 1.2, "accuracy": 3.5
        }
    }
    """

    def __init__(self, name: str = 'mobile_sensor_subscription_node'):
        super().__init__(name)

        self.declare_parameter('imu_topic', '')
        self.declare_parameter('mag_topic', '')
        self.declare_parameter('light_topic', '')
        self.declare_parameter('location_topic', '')
        self.declare_parameter('speed_topic', '')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('mag_frame_id', 'mag_link')
        self.declare_parameter('light_frame_id', 'light_link')
        self.declare_parameter('gps_frame_id', 'gps_link')
        self.declare_parameter('speed_frame_id', 'base_link')

        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.mag_topic = self.get_parameter('mag_topic').get_parameter_value().string_value
        self.light_topic = self.get_parameter('light_topic').get_parameter_value().string_value
        self.location_topic = self.get_parameter('location_topic').get_parameter_value().string_value
        self.speed_topic = self.get_parameter('speed_topic').get_parameter_value().string_value

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.mag_frame_id = self.get_parameter('mag_frame_id').get_parameter_value().string_value
        self.light_frame_id = self.get_parameter('light_frame_id').get_parameter_value().string_value
        self.gps_frame_id = self.get_parameter('gps_frame_id').get_parameter_value().string_value
        self.speed_frame_id = self.get_parameter('speed_frame_id').get_parameter_value().string_value

        self.imu_publisher = self._create_optional_publisher(Imu, self.imu_topic)
        self.mag_publisher = self._create_optional_publisher(MagneticField, self.mag_topic)
        self.light_publisher = self._create_optional_publisher(Illuminance, self.light_topic)
        self.location_publisher = self._create_optional_publisher(NavSatFix, self.location_topic)
        self.speed_publisher = self._create_optional_publisher(TwistStamped, self.speed_topic)

        if not any(
            [
                self.imu_publisher,
                self.mag_publisher,
                self.light_publisher,
                self.location_publisher,
                self.speed_publisher,
            ]
        ):
            self.get_logger().error(
                f"[{self.name}] No publish topics configured. "
                "Set at least one of imu_topic, mag_topic, light_topic, location_topic, speed_topic."
            )
            return

        self.mime = "test/json"
        self.create_timer(10, self.timer_callback)

    def _create_optional_publisher(self, msg_type, topic_name: str):
        if topic_name:
            return self.create_publisher(msg_type, topic_name, 10)
        return None

    def timer_callback(self):
        self.message.append(json.dumps("ping").encode('utf-8'))

    def _safe_float(self, value: Any) -> Optional[float]:
        try:
            if value is None:
                return None
            return float(value)
        except (TypeError, ValueError):
            return None

    def _fill_vector3(self, target, x: Optional[float], y: Optional[float], z: Optional[float]):
        if x is not None:
            target.x = x
        if y is not None:
            target.y = y
        if z is not None:
            target.z = z

    async def on_received(self, message: str):
        try:
            data = json.loads(message)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"[{self.name}] Invalid JSON payload: {exc}")
            return

        imu_payload = data.get('imuData') or data.get('imu') or {}
        location_payload = data.get('locationData') or data.get('location') or {}
        timestamp = self.get_clock().now().to_msg()

        if self.imu_publisher:
            self._publish_imu(timestamp, imu_payload)

        if self.mag_publisher:
            self._publish_magnetic_field(timestamp, imu_payload)

        if self.light_publisher:
            self._publish_light(timestamp, imu_payload, data)

        if self.location_publisher:
            self._publish_location(timestamp, location_payload)

        if self.speed_publisher:
            self._publish_speed(timestamp, location_payload, data)

    def _publish_imu(self, timestamp, imu_payload: dict):
        accel = (
            self._safe_float(imu_payload.get('accelX')),
            self._safe_float(imu_payload.get('accelY')),
            self._safe_float(imu_payload.get('accelZ')),
        )
        gyro = (
            self._safe_float(imu_payload.get('gyroX')),
            self._safe_float(imu_payload.get('gyroY')),
            self._safe_float(imu_payload.get('gyroZ')),
        )

        if not any(value is not None for value in (*accel, *gyro)):
            return

        msg = Imu()
        msg.header.stamp = timestamp
        msg.header.frame_id = self.frame_id

        self._fill_vector3(msg.linear_acceleration, *accel)
        self._fill_vector3(msg.angular_velocity, *gyro)

        # Orientation 정보를 받지 못하므로 -1로 표시
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

        self.imu_publisher.publish(msg)

    def _publish_magnetic_field(self, timestamp, imu_payload: dict):
        mag = (
            self._safe_float(imu_payload.get('magX')),
            self._safe_float(imu_payload.get('magY')),
            self._safe_float(imu_payload.get('magZ')),
        )

        if not any(value is not None for value in mag):
            return

        msg = MagneticField()
        msg.header.stamp = timestamp
        msg.header.frame_id = self.mag_frame_id

        self._fill_vector3(msg.magnetic_field, *mag)
        msg.magnetic_field_covariance[0] = -1.0
        self.mag_publisher.publish(msg)

    def _publish_light(self, timestamp, imu_payload: dict, root_payload: dict):
        value = self._safe_float(imu_payload.get('light'))
        if value is None:
            value = self._safe_float(root_payload.get('light'))

        if value is None:
            return

        msg = Illuminance()
        msg.header.stamp = timestamp
        msg.header.frame_id = self.light_frame_id
        msg.illuminance = value
        msg.variance = 0.0
        self.light_publisher.publish(msg)

    def _publish_location(self, timestamp, location_payload: dict):
        lat = self._safe_float(location_payload.get('latitude'))
        lon = self._safe_float(location_payload.get('longitude'))
        alt = self._safe_float(location_payload.get('altitude'))
        accuracy = self._safe_float(location_payload.get('accuracy'))

        if lat is None or lon is None:
            return

        msg = NavSatFix()
        msg.header.stamp = timestamp
        msg.header.frame_id = self.gps_frame_id
        msg.latitude = lat
        msg.longitude = lon
        if alt is not None:
            msg.altitude = alt

        if accuracy is not None and accuracy > 0.0:
            variance = accuracy * accuracy
            msg.position_covariance = [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            msg.status.status = NavSatStatus.STATUS_FIX
        else:
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            msg.status.status = NavSatStatus.STATUS_NO_FIX

        self.location_publisher.publish(msg)

    def _publish_speed(self, timestamp, location_payload: dict, root_payload: dict):
        speed = self._safe_float(location_payload.get('speed'))
        if speed is None:
            speed = self._safe_float(root_payload.get('speed'))

        if speed is None:
            return

        msg = TwistStamped()
        msg.header.stamp = timestamp
        msg.header.frame_id = self.speed_frame_id
        msg.twist.linear.x = speed
        self.speed_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    temp_node = Node('temp_node')
    temp_node.declare_parameter('name', 'mobile_sensor_subscription_node')
    name = temp_node.get_parameter('name').get_parameter_value().string_value
    temp_node.destroy_node()

    node = MobileSensorSubscriptionNode(name=name)

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


