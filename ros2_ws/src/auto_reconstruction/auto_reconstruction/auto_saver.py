#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import subprocess
from datetime import datetime

class AutoSaver(Node):
    def __init__(self):
        super().__init__('auto_saver')
        self.sub = self.create_subscription(Image, '/image_raw', self.callback, 10)
        self.bridge = CvBridge()
        self.save_dir = 'colmap_ws/images'
        os.makedirs(self.save_dir, exist_ok=True)
        self.counter = 0
        self.last_save_time = self.get_clock().now()
        self.interval = 1.0  # 每秒保存一张（按时间间隔）

    def callback(self, msg):
        now = self.get_clock().now()
        if (now - self.last_save_time).nanoseconds * 1e-9 >= self.interval:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            filename = os.path.join(self.save_dir, f'img_{self.counter:04d}.jpg')
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f'Saved {filename}')
            self.counter += 1
            self.last_save_time = now

def main(args=None):
    rclpy.init(args=args)
    node = AutoSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()