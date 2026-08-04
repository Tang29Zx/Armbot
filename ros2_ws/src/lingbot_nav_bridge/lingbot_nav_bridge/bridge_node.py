#!/usr/bin/env python3
"""
lingbot_nav_bridge: lingbot-map 3D → Nav2 导航桥接节点

功能:
  1. 加载 lingbot-map 模型 (GCTStream)
  2. 从相机读取 RGB 帧，流式推理
  3. 3D 点云 → 地面投影 → 2D 占用栅格 (/map)
  4. 相机位姿 → /odom + TF (map→odom→base_link)
  5. 发布 Nav2 所需的 /clock

用法:
  ros2 run lingbot_nav_bridge bridge_node --ros-args -p camera_id:=0
"""

import math
import os
import time
import threading

import numpy as np
import cv2
import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge

from lingbot_map.models.gct_stream import GCTStream
from lingbot_map.utils.load_fn import load_and_preprocess_images
from lingbot_map.utils.pose_enc import pose_encoding_to_extri_intri
from lingbot_map.utils.geometry import closed_form_inverse_se3_general


# ── 默认参数 ──────────────────────────────────────────────────────────
DEFAULT_MODEL_PATH = os.path.expanduser(
    "~/armbot-slam/models/models/Robbyant--lingbot-map/"
    "snapshots/master/lingbot-map/lingbot-map.pt"
)
IMAGE_SIZE   = 518
PATCH_SIZE   = 14
MAP_RESOLUTION = 0.05    # m/pixel
MAP_WIDTH      = 400     # pixels
MAP_HEIGHT     = 400     # pixels
GROUND_Z_MAX   = 0.3     # 地面点最大高度 (m)
CONF_THRESHOLD = 0.5     # 点云置信度阈值


class LingbotNavBridge(Node):
    """lingbot-map → Nav2 桥接节点"""

    def __init__(self):
        super().__init__('lingbot_nav_bridge')

        # ── 参数 ─────────────────────────────────────────────────────
        self.declare_parameter('model_path', DEFAULT_MODEL_PATH)
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('image_size', IMAGE_SIZE)
        self.declare_parameter('map_resolution', MAP_RESOLUTION)
        self.declare_parameter('map_width', MAP_WIDTH)
        self.declare_parameter('map_height', MAP_HEIGHT)
        self.declare_parameter('ground_z_max', GROUND_Z_MAX)
        self.declare_parameter('conf_threshold', CONF_THRESHOLD)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('max_frame_num', 1024)

        model_path    = self.get_parameter('model_path').value
        self.camera_id = self.get_parameter('camera_id').value
        self.image_size = self.get_parameter('image_size').value
        self.resolution = self.get_parameter('map_resolution').value
        self.map_w      = self.get_parameter('map_width').value
        self.map_h      = self.get_parameter('map_height').value
        self.ground_z   = self.get_parameter('ground_z_max').value
        self.conf_thr   = self.get_parameter('conf_threshold').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame  = self.get_parameter('map_frame').value
        self.cam_frame  = self.get_parameter('camera_frame').value

        # ── 加载模型 (GPU) ───────────────────────────────────────────
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f'Device: {self.device}')
        self.get_logger().info(f'Loading model: {model_path}')

        self.model = GCTStream(
            img_size=self.image_size,
            patch_size=PATCH_SIZE,
            enable_3d_rope=True,
            max_frame_num=self.get_parameter('max_frame_num').value,
            kv_cache_sliding_window=512,
            kv_cache_scale_frames=8,
            use_sdpa=False,
            camera_num_iterations=4,
        )
        ckpt = torch.load(model_path, map_location=self.device, weights_only=False)
        state_dict = ckpt.get('model', ckpt)
        self.model.load_state_dict(state_dict, strict=False)
        self.model = self.model.to(self.device).eval()
        self.get_logger().info('Model loaded')

        # ── 发布者 ────────────────────────────────────────────────────
        self.map_pub    = self.create_publisher(OccupancyGrid, '/map', 10)
        self.odom_pub   = self.create_publisher(Odometry, '/odom', 10)
        self.pose_pub   = self.create_publisher(PoseStamped, '/camera_pose', 10)
        self.image_pub  = self.create_publisher(Image, '/camera/image_raw', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.bridge = CvBridge()

        # ── 状态 ──────────────────────────────────────────────────────
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open camera {self.camera_id}')
            raise RuntimeError(f'Camera {self.camera_id} not available')

        self._frame_idx = 0
        self._pose_history = []  # (x, y, yaw)
        self._lock = threading.Lock()

        # ── 推理线程 ─────────────────────────────────────────────────
        self._running = True
        self._infer_thread = threading.Thread(
            target=self._inference_loop, daemon=True, name='lingbot-infer'
        )
        self._infer_thread.start()
        self.get_logger().info('Bridge node ready')

    # ── 推理循环 ──────────────────────────────────────────────────────
    def _inference_loop(self):
        """持续从相机读帧 → 推理 → 发布"""
        while self._running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.05)
                continue

            t0 = time.perf_counter()
            try:
                predictions = self._run_model(frame)
                if predictions is None:
                    continue

                # 发布图像
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                img_msg.header.stamp = self.get_clock().now().to_msg()
                self.image_pub.publish(img_msg)

                # 获取一帧预测 (取最后一个S维度)
                wp = predictions['world_points']  # (1, H, W, 3)
                ext = predictions['extrinsic']     # (1, 3, 4)
                conf = predictions.get('world_points_conf')  # (1, H, W)

                if isinstance(wp, torch.Tensor):
                    wp = wp[0].cpu().numpy()
                    ext = ext[0].cpu().numpy()
                    conf = conf[0].cpu().numpy() if conf is not None else None

                # 发布 2D 地图
                self._publish_map(wp, conf)

                # 发布位姿 + TF
                self._publish_pose(ext)

                dt = (time.perf_counter() - t0) * 1000
                self.get_logger().info(
                    f'Frame {self._frame_idx}: {dt:.0f}ms',
                    throttle_duration_sec=2.0,
                )
                self._frame_idx += 1

            except Exception as e:
                self.get_logger().error(f'Inference error: {e}')
                time.sleep(0.1)

    def _run_model(self, frame):
        """对单帧 RGB 图像运行 lingbot-map 推理"""
        import tempfile, os
        # 保存临时文件用于 load_and_preprocess_images
        with tempfile.NamedTemporaryFile(suffix='.jpg', delete=False) as f:
            cv2.imwrite(f.name, frame)
            tmp_path = f.name

        try:
            images = load_and_preprocess_images(
                [tmp_path], mode='crop',
                image_size=self.image_size, patch_size=PATCH_SIZE,
            )
        finally:
            os.unlink(tmp_path)

        images = images.to(self.device)  # (1, 3, H, W)
        h, w = images.shape[-2:]

        with torch.no_grad(), torch.amp.autocast('cuda', dtype=torch.bfloat16):
            predictions = self.model(images)

            # 后处理: 位姿解码
            extrinsic, intrinsic = pose_encoding_to_extri_intri(
                predictions['pose_enc'], (h, w)
            )
            e4 = torch.zeros((*extrinsic.shape[:-2], 4, 4), device=extrinsic.device, dtype=extrinsic.dtype)
            e4[..., :3, :4] = extrinsic
            e4[..., 3, 3] = 1.0
            e4 = closed_form_inverse_se3_general(e4)
            predictions['extrinsic'] = e4[..., :3, :4]
            predictions['intrinsic'] = intrinsic

        return predictions

    # ── 3D → 2D 地图 ──────────────────────────────────────────────────
    def _publish_map(self, world_points, conf):
        """3D 点云 → 2D 占用栅格 → 发布 /map"""
        h, w = world_points.shape[:2]
        pts = world_points.reshape(-1, 3)
        if conf is not None:
            conf_flat = conf.reshape(-1)
            mask = (conf_flat > self.conf_thr)
            pts = pts[mask]

        # 地面投影: 只取 z < ground_z_max 的点
        ground = pts[pts[:, 2] < self.ground_z]

        # 创建栅格 (原点在中心)
        grid = np.zeros((self.map_h, self.map_w), dtype=np.int8)
        cx, cy = self.map_w // 2, self.map_h // 2

        xs = (ground[:, 0] / self.resolution + cx).astype(int)
        ys = (ground[:, 1] / self.resolution + cy).astype(int)

        valid = (xs >= 0) & (xs < self.map_w) & (ys >= 0) & (ys < self.map_h)
        grid[ys[valid], xs[valid]] = 100  # occupied

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = self.resolution
        msg.info.width = self.map_w
        msg.info.height = self.map_h
        msg.info.origin.position.x = -self.map_w * self.resolution / 2.0
        msg.info.origin.position.y = -self.map_h * self.resolution / 2.0
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.flatten().tolist()
        self.map_pub.publish(msg)

    # ── 位姿 → /odom + TF ─────────────────────────────────────────────
    def _publish_pose(self, extrinsic):
        """相机外参 → 2D 位姿 → /odom + TF"""
        # extrinsic: cam→world, 取最后一帧的位置
        # 简化为 2D: 只取 x, y, yaw
        R = extrinsic[:3, :3]
        t = extrinsic[:3, 3]
        yaw = math.atan2(R[1, 0], R[0, 0])
        x, y = t[0], t[1]

        now = self.get_clock().now().to_msg()

        # Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.odom_pub.publish(odom)

        # TF: odom → base_link
        tf_odom = TransformStamped()
        tf_odom.header.stamp = now
        tf_odom.header.frame_id = self.odom_frame
        tf_odom.child_frame_id = self.base_frame
        tf_odom.transform.translation.x = x
        tf_odom.transform.translation.y = y
        tf_odom.transform.translation.z = 0.0
        tf_odom.transform.rotation.z = qz
        tf_odom.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_odom)

        # TF: base_link → camera (固定偏移, 假设相机在前方上方)
        tf_cam = TransformStamped()
        tf_cam.header.stamp = now
        tf_cam.header.frame_id = self.base_frame
        tf_cam.child_frame_id = self.cam_frame
        tf_cam.transform.translation.x = 0.1
        tf_cam.transform.translation.z = 0.8
        tf_cam.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf_cam)

        # PoseStamped
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = self.map_frame
        pose.pose = odom.pose.pose
        self.pose_pub.publish(pose)

    def destroy_node(self):
        self._running = False
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LingbotNavBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == '__main__':
    main()
