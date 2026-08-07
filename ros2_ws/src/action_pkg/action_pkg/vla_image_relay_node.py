"""Rate-limit the latest compressed camera frame before cross-host DDS."""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class VlaImageRelayNode(Node):
    def __init__(self):
        super().__init__("vla_image_relay_node")
        self.declare_parameter("input_topic", "/image")
        self.declare_parameter("output_topic", "/vla/image")
        self.declare_parameter("output_rate_hz", 10.0)
        self.declare_parameter("max_input_age_sec", 0.5)

        rate = float(self.get_parameter("output_rate_hz").value)
        max_age = float(self.get_parameter("max_input_age_sec").value)
        if rate <= 0.0:
            raise ValueError("output_rate_hz must be positive")
        if max_age <= 0.0:
            raise ValueError("max_input_age_sec must be positive")

        self._max_age = max_age
        self._latest = None
        self._latest_received_at = None
        self._generation = 0
        self._published_generation = 0
        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._publisher = self.create_publisher(
            CompressedImage, output_topic, qos_profile_sensor_data
        )
        self._subscription = self.create_subscription(
            CompressedImage,
            input_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self._timer = self.create_timer(1.0 / rate, self._publish_latest)
        self.get_logger().info(
            "relaying latest compressed frame %s -> %s at <= %.1f Hz"
            % (input_topic, output_topic, rate)
        )

    def _on_image(self, message):
        self._latest = message
        self._latest_received_at = time.monotonic()
        self._generation += 1

    def _publish_latest(self):
        if self._latest is None or self._latest_received_at is None:
            return
        if self._generation == self._published_generation:
            return
        if time.monotonic() - self._latest_received_at > self._max_age:
            return
        self._publisher.publish(self._latest)
        self._published_generation = self._generation


def main(args=None):
    rclpy.init(args=args)
    node = VlaImageRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
