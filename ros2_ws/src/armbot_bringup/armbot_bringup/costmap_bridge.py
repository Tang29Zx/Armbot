#!/usr/bin/env python3
"""
costmap_bridge.py - Nav2 Costmap -> OccupancyGrid 桥接

Nav2 Humble 的 /global_costmap/costmap_raw 是 nav2_msgs/msg/Costmap 类型，
而 explore_lite（m-explore-ros2，Galactic 时代代码）订阅 nav_msgs/msg/OccupancyGrid。
本节点做类型转换桥接，使 explore_lite 能正常获取 costmap。
"""

import rclpy
from rclpy.node import Node
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid


class CostmapBridge(Node):
    def __init__(self):
        super().__init__('costmap_bridge')
        self.sub = self.create_subscription(
            Costmap, '/global_costmap/costmap_raw', self.cb, 10)
        self.pub = self.create_publisher(
            OccupancyGrid, '/explore/costmap', 10)
        self.get_logger().info(
            'Bridge: /global_costmap/costmap_raw (Costmap) -> /explore/costmap (OccupancyGrid)')

    def cb(self, msg: Costmap):
        og = OccupancyGrid()
        og.header = msg.header
        og.info.map_load_time = msg.metadata.map_load_time
        og.info.resolution = msg.metadata.resolution
        og.info.width = msg.metadata.size_x
        og.info.height = msg.metadata.size_y
        og.info.origin.position.x = msg.metadata.origin.position.x
        og.info.origin.position.y = msg.metadata.origin.position.y
        og.info.origin.position.z = 0.0
        og.info.origin.orientation = msg.metadata.origin.orientation
        # Nav2 costmap: 0=free, 1-252=inflation, 253=inscribed, 254=lethal, 255=unknown
        # OccupancyGrid: -1=unknown, 0=free, 1-100=occupied
        og.data = [
            -1 if v >= 255 else 100 if v >= 254 else min(int(v), 100)
            for v in msg.data
        ]
        self.pub.publish(og)


def main(args=None):
    rclpy.init(args=args)
    node = CostmapBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == '__main__':
    main()
