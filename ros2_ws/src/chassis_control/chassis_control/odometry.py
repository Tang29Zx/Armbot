#!/usr/bin/env python3
"""
odometry.py - 麦克纳姆轮里程计（编码器闭环）
对应 STM32 odometry.c 的 Python 实现

坐标系约定（车体）：
  前 → +X, 左 → +Y, 逆时针 → +Yaw

依赖：chassis_control.motor_driver.MotorDriver
"""

import math
import struct
import time
from dataclasses import dataclass
from typing import List, Optional

# ─── 物理参数（与 odometry.h 一致）────────────────────────────────
WHEEL_RADIUS   = 0.0325      # 轮子半径 (m)
WHEEL_DIST_LR  = 0.130       # 左右轮中心距 (m)，轮距的一半
WHEEL_DIST_FB  = 0.130       # 前后轮中心距 (m)，轴距的一半
ENCODER_PPR    = 3924         # 标定值 (2026-08-17 实测验证：推车 1.0m=odom 0.479m 是命令超时未走满，实际 0.5m 一致，PPR 保持原值)

# 运动学常数（预计算）
_KIN_K = WHEEL_RADIUS / 4.0
_KIN_W = _KIN_K / (WHEEL_DIST_LR + WHEEL_DIST_FB) * 1.315   # 修复：分母不再额外 /2；8-18 整圈校准（odom 367°=实际 330°）→ ×1.315

# ─── 数据结构 ──────────────────────────────────────────────────


@dataclass
class OdoPose:
    """里程计位姿（与 OdoPoseTypeDef 对应）"""
    x: float = 0.0        # 世界 X (m)，前为正
    y: float = 0.0        # 世界 Y (m)，左为正
    theta: float = 0.0    # 航向角 (rad)，逆时针为正
    vx: float = 0.0       # 车体前进速度 (m/s)
    vy: float = 0.0       # 车体横向速度 (m/s)
    wz: float = 0.0       # 角速度 (rad/s)
    stamp: float = 0.0     # 时间戳 (s)

    def __repr__(self):
        return (f"x={self.x:8.4f} y={self.y:8.4f} θ={self.theta:7.3f}rad "
                f"vx={self.vx:6.3f} vy={self.vy:6.3f} wz={self.wz:6.3f}")

    def to_dict(self):
        return {
            "x": self.x, "y": self.y, "theta": self.theta,
            "vx": self.vx, "vy": self.vy, "wz": self.wz,
            "stamp": self.stamp
        }

    def to_csv_header(self):
        return "stamp,x,y,theta,vx,vy,wz"

    def to_csv_row(self):
        return f"{self.stamp:.3f},{self.x:.6f},{self.y:.6f},{self.theta:.6f},{self.vx:.6f},{self.vy:.6f},{self.wz:.6f}"


# ─── 里程计类 ──────────────────────────────────────────────────


class Odometry:
    """
    麦克纳姆轮里程计

    用法:
        from rdk_motor_i2c_test import MotorDriver
        md = MotorDriver(bus_num=5)
        odo = Odometry(md)

        while True:
            odo.update()
            print(odo.pose)
            time.sleep(0.02)
    """

    def __init__(self, motor_driver, sample_dt: float = 0.020, enable_kalman: bool = False):
        """
        Parameters
        ----------
        motor_driver : MotorDriver
            I2C 电机驱动板实例（至少需要 get_encoder() 方法）
        sample_dt : float
            调用 update() 的周期 (s)，默认 20ms = 50Hz
        enable_kalman : bool
            是否启用简单卡尔曼滤波（需 QMI8658 IMU 数据）
        """
        self._md = motor_driver
        self._dt = sample_dt
        self._kalman = enable_kalman

        self.pose = OdoPose()
        self._last_enc: List[int] = [0, 0, 0, 0]
        self._inited = False

        # 卡尔曼滤波状态（简化 1D 对 x/y/theta 分别滤波）
        if enable_kalman:
            self._kf_p = [0.1, 0.1, 0.1]     # 估计协方差
            self._kf_q = [0.001, 0.001, 0.001]  # 过程噪声
            self._kf_r = [0.01, 0.01, 0.01]    # 测量噪声

        self._frame_count = 0

    # ─── 初始化 ──────────────────────────────────────────────

    def init(self):
        """初始化里程计，预读编码器基准值"""
        enc = self._md.get_encoder()
        self._last_enc = list(enc)
        self.pose = OdoPose()
        self._inited = True
        self._frame_count = 0
        print(f"[ODO] 初始化完成, 编码器基线: {self._last_enc}")

    # ─── 核心更新 ────────────────────────────────────────────

    def update(self):
        """
        里程计更新（对应 odometry_update）
        调用频率：50Hz (每 20ms)
        """
        if not self._inited:
            self.init()
            return False

        # 1. 读取编码器
        enc = self._md.get_encoder()
        if not enc:
            return False

        # 2. 编码器差分 → 轮角位移 (rad)
        dtheta = []
        for i in range(4):
            delta = enc[i] - self._last_enc[i]
            # 8-22: 编码器跳变保护——单帧 delta 超 500（正常直行 ~20-30）判定读数错误，该轮忽略
            if abs(delta) > 500:
                dtheta.append(0.0)
            else:
                dtheta.append(delta / ENCODER_PPR * 2.0 * math.pi)
            self._last_enc[i] = enc[i]

        # 3. 麦克纳姆正向运动学 → 底盘速度
        # 轮序: [0]=M1右前  [1]=M2右后  [2]=M3左前  [3]=M4左后
        vx = _KIN_K * (dtheta[0] - dtheta[1] - dtheta[2] + dtheta[3]) / self._dt
        vy = _KIN_K * (dtheta[0] + dtheta[1] + dtheta[2] + dtheta[3]) / self._dt
        wz = -_KIN_W * (-dtheta[0] + dtheta[1] - dtheta[2] + dtheta[3]) / self._dt  # 8-18: 翻转符号修旋转方向（cmd +z 应报正）

        # 4. 位姿积分（欧拉法）
        self.pose.theta += wz * self._dt
        self.pose.theta = _norm_angle(self.pose.theta)

        ct = math.cos(self.pose.theta)
        st = math.sin(self.pose.theta)

        self.pose.x += (vx * ct - vy * st) * self._dt
        self.pose.y += (vx * st + vy * ct) * self._dt
        self.pose.vx = vx
        self.pose.vy = vy
        self.pose.wz = wz
        self.pose.stamp = time.time()

        self._frame_count += 1
        return True

    # ─── 工具 ──────────────────────────────────────────────────

    def reset(self):
        """重置位姿到原点"""
        self.pose = OdoPose()
        self._inited = False  # 强制下次 update 重新 init

    def is_inited(self) -> bool:
        return self._inited

    @property
    def frame_count(self) -> int:
        return self._frame_count

    # ─── 卡尔曼滤波（可选）────────────────────────────────────

    def update_with_imu(self, imu_wz: float):
        """
        带 IMU 补充的里程计更新
        imu_wz : float  IMU 陀螺仪角速度 (rad/s)

        使用简化卡尔曼滤波融合编码器 wz 与 IMU wz
        """
        if not self.update():
            return False

        # 简化 Kalman: 仅对 theta 做融合
        # 预测：θ_k = θ_{k-1} + IMU_wz * dt
        theta_pred = _norm_angle(self.pose.theta + imu_wz * self._dt)
        # 测量：编码器推导的 θ（已在 self.pose.theta 中）
        theta_meas = self.pose.theta

        # 卡尔曼增益
        k = self._kf_p[2] / (self._kf_p[2] + self._kf_r[2])
        # 更新
        self.pose.theta = theta_pred + k * (theta_meas - theta_pred)
        self.pose.theta = _norm_angle(self.pose.theta)
        self._kf_p[2] = (1 - k) * self._kf_p[2] + self._kf_q[2]

        return True


# ─── 工具函数 ──────────────────────────────────────────────────


def _norm_angle(a: float) -> float:
    """角度归一化到 [-π, π]"""
    return math.atan2(math.sin(a), math.cos(a))
