#!/usr/bin/env python3
"""
chassis_control_node.py - LeArm 麦克纳姆底盘 ROS2 节点

功能：
  - 订阅 /cmd_vel (geometry_msgs/Twist)，转换为 4 路电机速度
  - 读取编码器，计算里程计，发布 /odom (nav_msgs/Odometry)
  - 发布 tf：odom -> base_link

用法：
  ros2 run chassis_control chassis_control_node
"""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from chassis_control.motor_driver import MotorDriver
from chassis_control.odometry import Odometry as Odo


# ─── 麦克纳姆逆运动学 ────────────────────────────────────────

def mecanum_inverse(angle_deg, speed, rot, drift=False):
    """
    与 interactive_odo.py / rdk_motor_i2c_test.py 一致的逆运动学
    angle_deg : 0=右, 90=前, 180=左, 270=后
    speed, rot: 0~100 的归一化速度
    """
    rad = angle_deg * math.pi / 180
    # 旋转分量不砍半（原 sf=0.5 导致旋转执行只有指令一半 → 转向严重不足 → 地图畸变）
    sf = 1.0
    s = speed / math.sqrt(2)
    s_sin, s_cos = s * math.sin(rad), s * math.cos(rad)

    if drift:
        m1 = s_sin - s_cos + rot * 2 * sf
        m2 = -(s_sin + s_cos) - rot * 2 * sf
        m3 = -(s_sin + s_cos)
        m4 = s_sin - s_cos
    else:
        m1 = s_sin - s_cos + rot * sf
        m2 = -(s_sin + s_cos) - rot * sf
        m3 = -(s_sin + s_cos) + rot * sf
        m4 = s_sin - s_cos - rot * sf

    return [max(-100, min(100, int(m))) for m in [m1, m2, m3, m4]]


# ─── Twist → 电机速度 ────────────────────────────────────────

def twist_to_motor_speeds(twist, max_linear, max_angular):
    """
    将 geometry_msgs/Twist 转换为 4 路电机速度 [-100, 100]
    """
    vx = twist.linear.x
    vy = twist.linear.y
    omega = twist.angular.z

    # 线速度大小与方向角
    linear = math.sqrt(vx * vx + vy * vy)
    if linear > max_linear:
        linear = max_linear

    if linear < 1e-6:
        angle = 90.0  # 无平移时默认朝前，避免 atan2 跳变
    else:
        # 90°=前，90°=左，所以 angle = atan2(vx, -vy)
        angle = math.atan2(vx, -vy) * 180 / math.pi

    speed = linear / max_linear * 100.0
    rot = omega / max_angular * 100.0
    rot = max(-100, min(100, rot))

    return mecanum_inverse(angle, speed, rot), angle, speed, rot


# ─── ROS2 节点 ───────────────────────────────────────────────

class LeArmChassisNode(Node):
    def __init__(self):
        super().__init__('chassis_control')

        # ── 参数 ──
        self.declare_parameter('i2c_bus', 5)
        self.declare_parameter('i2c_addr', 0x34)
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('wheel_dist_lr', 0.130)
        self.declare_parameter('wheel_dist_fb', 0.130)

        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.i2c_addr = self.get_parameter('i2c_addr').value
        self.update_rate = self.get_parameter('update_rate').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # ── 初始化硬件 ──
        self.get_logger().info(
            f"Connecting to motor driver at I2C bus={self.i2c_bus}, addr=0x{self.i2c_addr:02X}")
        self.driver = MotorDriver(bus_num=self.i2c_bus, addr=self.i2c_addr)
        self.driver.init()
        self.get_logger().info("Motor driver initialized.")

        self.odo = Odo(self.driver, sample_dt=1.0/self.update_rate)
        self.odo.init()

        # ── 订阅 /cmd_vel ──
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 0.5  # 秒
        self.current_speeds = [0, 0, 0, 0]
        self.target_speeds = [0, 0, 0, 0]
        self._frame = 0

        # ── 发布 /odom ──
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── 定时器 ──
        self.dt = 1.0 / self.update_rate
        self.timer = self.create_timer(self.dt, self.update)

        self.get_logger().info("LeArm chassis node started.")

    # ── /cmd_vel 回调 ──

    def cmd_vel_callback(self, msg: Twist):
        motor_speeds, angle, speed, rot = twist_to_motor_speeds(
            msg, self.max_linear, self.max_angular)
        if motor_speeds != self.target_speeds:
            self.get_logger().info(
                f"CMD: vx={msg.linear.x:.2f} wz={msg.angular.z:.2f} -> motor={motor_speeds}")
        self.target_speeds = motor_speeds
        self.last_cmd_time = self.get_clock().now()
        self.get_logger().debug(
            f"cmd_vel: vx={msg.linear.x:.2f} vy={msg.linear.y:.2f} "
            f"wz={msg.angular.z:.2f} -> motor={motor_speeds} "
            f"(angle={angle:.1f}, speed={speed:.1f}, rot={rot:.1f})")

    # ── 主循环 ──

    def update(self):
        now = self.get_clock().now()

        # 超时检测：长时间没收到 /cmd_vel 就强制停车（8-18: 改用 emergency_stop 写 3 遍，确保驱动板真正停车）
        if (now - self.last_cmd_time).nanoseconds / 1e9 > self.cmd_timeout:
            if self.target_speeds != [0, 0, 0, 0] or self.current_speeds != [0, 0, 0, 0]:
                self.get_logger().warn("Cmd_vel timeout, forcing stop.")
                self.driver.emergency_stop()
            self.target_speeds = [0, 0, 0, 0]
            self.current_speeds = [0, 0, 0, 0]

        # 电机速度写入（立即写入，不渐变）
        if self.target_speeds != self.current_speeds:
            self.driver.set_speed(self.target_speeds)
            self.get_logger().info(f"SETSPEED: {self.target_speeds}")
            self.current_speeds = list(self.target_speeds)

        # 更新里程计
        self.odo.update()
        p = self.odo.pose

        # ── 临时旋转调试日志（每 50 帧）──
        if self._frame % 50 == 0:
            enc = self.driver.get_encoder()
            self.get_logger().info(
                f"[DBG] enc={enc} theta={p.theta:.3f} vx={p.vx:.3f} vy={p.vy:.3f} wz={p.wz:.3f}")
        self._frame += 1

        # 发布 /odom
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = p.x
        odom_msg.pose.pose.position.y = p.y
        odom_msg.pose.pose.position.z = 0.0

        # 航向角 → 四元数
        qz = math.sin(p.theta / 2.0)
        qw = math.cos(p.theta / 2.0)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        odom_msg.twist.twist.linear.x = p.vx
        odom_msg.twist.twist.linear.y = p.vy
        odom_msg.twist.twist.angular.z = p.wz

        self.odom_pub.publish(odom_msg)

        # 发布 tf
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = p.x
            t.transform.translation.y = p.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

    # ── 销毁 ──

    def destroy_node(self):
        self.get_logger().info("Stopping motors and closing I2C.")
        self.driver.emergency_stop()
        self.driver.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LeArmChassisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
