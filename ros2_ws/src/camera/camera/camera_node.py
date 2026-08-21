"""
Camera node for RDK X5.

Publishes /image_raw (sensor_msgs/Image) from the onboard camera.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraNode(Node):
    """ROS 2 node wrapping the RDK X5 camera."""

    def __init__(self):
        super().__init__('camera_node')

        # Parameters
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('frame_id', 'camera_link')

        self.camera_id = self.get_parameter('camera_id').value
        self.frame_id = self.get_parameter('frame_id').value
        publish_rate = self.get_parameter('publish_rate').value

        # Publishers
        self.image_pub = self.create_publisher(Image, '/image_raw', 10)

        # CV bridge
        self.bridge = CvBridge()

        # Open camera
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera {self.camera_id}')
            raise RuntimeError(f'Cannot open camera {self.camera_id}')

        self.get_logger().info(f'Camera {self.camera_id} opened')

        # Publishing timer
        self.timer = self.create_timer(1.0 / publish_rate, self._publish_frame)

    def _publish_frame(self):
        """Capture and publish one frame."""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read camera frame', throttle_duration_sec=5.0)
            return

        # Convert OpenCV BGR to ROS Image
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.image_pub.publish(msg)

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CameraNode()
        rclpy.spin(node)
    except RuntimeError as e:
        print(f'Camera node failed: {e}')
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
