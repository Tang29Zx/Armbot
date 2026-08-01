#!/usr/bin/env python3
"""
Armbot IK workspace probe.

用于探测机械臂逆运动学的可达工作空间。

发送一组 MODE_END_EFFECTOR (mode=1) 坐标点，逐一判断固件逆解是否可解
（OK / NO_SOLVE / TIMEOUT）。坐标单位为厘米（cm），与固件约定一致；
pitch 保持 0.0（其单位尚未与固件确认）。

运行（在 RDK 上，需先 source ROS2 + workspace 环境）：
  source /opt/ros/humble/setup.bash
  source ~/Armbot/ros2_ws/install/setup.bash
  python3 ~/Armbot/ros2_ws/src/action_pkg/action_pkg/ik_probe.py

坐标单位为厘米(cm)，与固件约定一致（kinematics.h 注释「单位：cm」）；
pitch 为度(°)，此处固定 0.0（参考例程用 0 / -40）。依 5.2.7 例程，
可达工作空间大致为 x≈20cm（正前方）、y≈±15cm、z≈5~20cm，故扫点
围绕该区域而非原点。

重要：每个探测点之前先发一个 STOP (mode=0)。原因——固件一旦回
NO_SOLVE，节点会停在 STATE_ERROR 且固件状态字符串卡在 'NO_SOLVE'，
节点只在「状态跳变」时处理（见 arm_controller_node.poll_status），
连续 NO_SOLVE 会被吞掉、误判为 TIMEOUT。STOP 让固件回到 STOP_OK_，
下一个点的 NO_SOLVE 才是一次真实跳变，可被正确识别。
"""
import rclpy
from rclpy.node import Node
from action_interfaces.msg import ArmCommand, ArmState

ERR_FW_NO_SOLVE = 0x0020
ERR_CMD_TIMEOUT = 0x0016


class IKProbe(Node):
    def __init__(self):
        super().__init__('ik_probe')
        self.cmd_pub = self.create_publisher(ArmCommand, '/arm/command', 10)
        self.sub = self.create_subscription(ArmState, '/arm/state', self._on_state, 10)
        self._state = None
        self._seq = 1000  # 从大数起，避免与手工命令（1~n）冲突

    def _on_state(self, msg):
        self._state = msg

    def _now_ns(self):
        return self.get_clock().now().nanoseconds

    def _spin_until(self, pred, timeout_sec):
        deadline = self._now_ns() + int(timeout_sec * 1e9)
        while self._now_ns() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if pred():
                return True
        return False

    def probe(self, x, y, z, pitch=0.0, dur=2.0):
        # 1) STOP 清错 + 让固件状态离开 NO_SOLVE
        self._seq += 1
        stop = ArmCommand()
        stop.mode = ArmCommand.MODE_STOP
        stop.sequence_id = self._seq
        self.cmd_pub.publish(stop)
        self._spin_until(
            lambda: self._state is not None and self._state.state == ArmState.STATE_IDLE,
            1.0)

        # 2) 发送目标点
        self._seq += 1
        expecting = self._seq
        cmd = ArmCommand()
        cmd.mode = ArmCommand.MODE_END_EFFECTOR
        cmd.x, cmd.y, cmd.z = float(x), float(y), float(z)
        cmd.pitch = float(pitch)
        cmd.duration_sec = float(dur)
        cmd.sequence_id = expecting
        self.cmd_pub.publish(cmd)

        # 3) 等待终态（SUCCEEDED 或 ERROR），按 sequence_id 过滤
        deadline = self._now_ns() + int((dur + 4.0) * 1e9)
        while self._now_ns() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            s = self._state
            if s is None or s.sequence_id != expecting:
                continue
            if s.state == ArmState.STATE_SUCCEEDED:
                return 'OK'
            if s.state == ArmState.STATE_ERROR:
                ec = s.error_code
                if ec == ERR_FW_NO_SOLVE:
                    return 'NO_SOLVE'
                if ec == ERR_CMD_TIMEOUT:
                    return 'TIMEOUT'
                return 'ERR(0x%04X)' % ec
        return 'TIMEOUT'


def main():
    rclpy.init()
    node = IKProbe()

    # 探测点（单位 cm，原点在底座，z 向上）。依 5.2.7 例程，可达区约
    # x≈20（正前方）、y≈±15、z≈5~20。扫点围绕该区域边界展开，用于
    # 实测最小/最大可达半径。按需要自行增删。
    vertical = [(20, 0, z) for z in (5, 8, 10, 12, 15, 18, 20, 23)]
    horiz_x = [(x, 0, 15) for x in (10, 15, 18, 20, 22, 25)]
    horiz_y = [(20, y, 15) for y in (-15, -10, -5, 0, 5, 10, 15)]

    print('=== 竖直扫描 (x=0,y=0) ===')
    for (x, y, z) in vertical:
        print('  (%3d,%3d,%3d) -> %s' % (x, y, z, node.probe(x, y, z)))

    print('=== 水平 x 扫描 (y=0,z=12) ===')
    for (x, y, z) in horiz_x:
        print('  (%3d,%3d,%3d) -> %s' % (x, y, z, node.probe(x, y, z)))

    print('=== 水平 y 扫描 (x=0,z=12) ===')
    for (x, y, z) in horiz_y:
        print('  (%3d,%3d,%3d) -> %s' % (x, y, z, node.probe(x, y, z)))

    # 结束时停车
    node._seq += 1
    stop = ArmCommand()
    stop.mode = ArmCommand.MODE_STOP
    stop.sequence_id = node._seq
    node.cmd_pub.publish(stop)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
