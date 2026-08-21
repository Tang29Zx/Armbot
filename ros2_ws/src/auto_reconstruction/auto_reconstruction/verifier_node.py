#!/usr/bin/env python3
"""
YOLO26 medicine-box verifier node (RDK X5 side).

Subscribes to /image_raw, runs YOLO26 detection, and publishes
/YoloDetection messages.  Also publishes an annotated debug image.

Calibrated workflow (see vla-module.md sec 5):
  1. Place medicine box at the fixed target position.
  2. Drive the robot to the fixed nav point.
  3. Run the calibration routine (record bbox 3×, average → CALIBRATED_BBOX).
  4. At runtime: verify() → compare against CALIBRATED_BBOX → publish offsets.
"""

import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

from action_interfaces.msg import YoloDetection

from auto_reconstruction.yolo_detector import MedicineBoxDetector
from auto_reconstruction.model_utils import load_calibrated_bbox


class YoloVerifierNode(Node):
    """ROS 2 node wrapping MedicineBoxDetector.

    Publishes:
      /yolo/detection  (YoloDetection)  — structured detection result
      /yolo/annotated  (Image)           — debug image with bbox overlay
    """

    def __init__(self):
        super().__init__('yolo_verifier_node')

        # --- Parameters ---
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('target_class', 'medicine_box')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('calib_file', '')  # path to calibrated bbox yaml

        model_path = self.get_parameter('model_path').value
        conf_thresh = self.get_parameter('confidence_threshold').value
        target_class = self.get_parameter('target_class').value
        image_topic = self.get_parameter('image_topic').value
        device = self.get_parameter('device').value
        self.publish_annotated = self.get_parameter('publish_annotated').value
        calib_file = self.get_parameter('calib_file').value

        if not model_path:
            self.get_logger().error(
                'model_path parameter is required. '
                'Set it via command line or YAML config.'
            )
            raise ValueError('model_path is empty')

        # --- Load calibrated bbox ---
        calibrated_bbox = None
        if calib_file:
            calibrated_bbox = load_calibrated_bbox(calib_file)
            if calibrated_bbox:
                self.get_logger().info(f'Loaded calibrated bbox: {calibrated_bbox}')
            else:
                self.get_logger().warn(f'Failed to load calib from {calib_file}')

        # --- Detector ---
        self.get_logger().info(f'Loading model: {model_path}')
        self.detector = MedicineBoxDetector(
            model_path=model_path,
            confidence_threshold=conf_thresh,
            target_class=target_class,
            device=device,
            calibrated_bbox=calibrated_bbox,
        )
        self.get_logger().info('Model loaded successfully')

        # --- CV bridge ---
        self.bridge = CvBridge()

        # --- Subscribers ---
        self.image_sub = self.create_subscription(
            Image, image_topic, self._image_callback, 10
        )

        # --- Publishers ---
        self.detection_pub = self.create_publisher(
            YoloDetection, '/yolo/detection', 10
        )

        if self.publish_annotated:
            self.annotated_pub = self.create_publisher(
                Image, '/yolo/annotated', 10
            )

        self.get_logger().info(
            f'YOLO verifier ready. Listening on {image_topic}, '
            f'target={target_class}, conf={conf_thresh}'
        )

    def _image_callback(self, msg: Image):
        """Process incoming image: detect → publish.

        Non-blocking path; detection runs synchronously in the callback.
        If inference latency becomes a problem, switch to a timer-driven
        design that drops frames when busy.
        """
        try:
            # Convert ROS Image → numpy
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Run detection
            detection = self.detector.detect(cv_image)

            # Verify against calibrated position (adds warning if shifted)
            detection = self.detector.verify_fixed_position(detection)

            # Publish structured result
            self._publish_detection(msg.header, detection)

            # Publish annotated debug image
            if self.publish_annotated:
                annotated = MedicineBoxDetector.annotate_image(cv_image, detection)
                anno_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                anno_msg.header = msg.header
                self.annotated_pub.publish(anno_msg)

        except Exception as e:
            self.get_logger().error(
                f'Detection failed: {e}', throttle_duration_sec=2.0
            )

    def _publish_detection(self, header: Header, detection: dict):
        """Convert detection dict → YoloDetection message and publish."""
        out = YoloDetection()
        out.header = header
        out.confirmed = detection['confirmed']
        out.class_name = detection.get('class_name', '')
        out.x1 = detection.get('x1', 0.0)
        out.y1 = detection.get('y1', 0.0)
        out.x2 = detection.get('x2', 0.0)
        out.y2 = detection.get('y2', 0.0)
        out.offset_x = detection.get('offset_x', 0.0)
        out.offset_y = detection.get('offset_y', 0.0)
        out.confidence = detection.get('confidence', 0.0)
        out.image_width = detection.get('image_width', 0)
        out.image_height = detection.get('image_height', 0)

        self.detection_pub.publish(out)

        # Log at info level only when confirmed (avoid spam)
        status = 'CONFIRMED' if out.confirmed else 'no detection'
        warn = detection.get('warning', '')
        if warn:
            self.get_logger().warn(
                f'{status}  conf={out.confidence:.2f}  offset=({out.offset_x:.0f},{out.offset_y:.0f})  warn={warn}',
                throttle_duration_sec=1.0,
            )
        else:
            self.get_logger().info(
                f'{status}  conf={out.confidence:.2f}  offset=({out.offset_x:.0f},{out.offset_y:.0f})',
                throttle_duration_sec=2.0,
            )


def main(args=None):
    rclpy.init(args=args)
    try:
        node = YoloVerifierNode()
        rclpy.spin(node)
    except (ValueError, RuntimeError) as e:
        print(f'YoloVerifierNode startup failed: {e}')
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
