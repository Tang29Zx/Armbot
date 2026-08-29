#!/usr/bin/env python3
"""map_relay: 从 map_server 的 GetMap service 取完整静态图，发布到 /map_static。
背景：/map 有 2 个发布者（map_server 完整图 + slam_toolbox 局部图，publish_map:false
不生效），订阅 /map 会被 slam 高频局部图淹没，永远等不到完整图（曾导致 global
costmap 被缩成 147x31 小窗、远处目标无法规划）。
方案：改用 service（/map_server/map, nav_msgs/srv/GetMap）一次性取图——
不订阅话题，无竞争，绝对可靠。发布 /map_static（TRANSIENT_LOCAL latched）
供 costmap 静态层 + Web 使用。
"""
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap

MAP_QOS = rclpy.qos.QoSProfile(
    depth=1,
    reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
    durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
)
SERVICE_NAME = '/map_server/map'


class MapRelay(Node):
    def __init__(self):
        super().__init__('map_relay')
        self.pub = self.create_publisher(OccupancyGrid, '/map_static', MAP_QOS)
        self.cli = self.create_client(GetMap, SERVICE_NAME)
        self.done = False
        self.retry_at = time.time() + 5.0   # 等 map_server lifecycle active
        self.create_timer(1.0, self.tick)

    def tick(self):
        if self.done or time.time() < self.retry_at:
            return
        if not self.cli.wait_for_service(timeout_sec=0.5):
            print('[map_relay] 等待 %s 可用...' % SERVICE_NAME, flush=True)
            self.retry_at = time.time() + 3.0
            return
        print('[map_relay] 调用 GetMap service...', flush=True)
        fut = self.cli.call_async(GetMap.Request())
        fut.add_done_callback(self.got)
        self.retry_at = time.time() + 3600   # 防止重复调用

    def got(self, fut):
        try:
            m = fut.result().map
            w, h = m.info.width, m.info.height
            if w * h == 0:
                raise ValueError('空地图')
            self.pub.publish(m)
            self.done = True
            print('[map_relay] 已发布完整图 %dx%d 到 /map_static' % (w, h), flush=True)
        except Exception as e:
            print('[map_relay] GetMap 失败: %s（重试）' % e, flush=True)
            self.retry_at = time.time() + 3.0


def main():
    rclpy.init()
    n = MapRelay()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
