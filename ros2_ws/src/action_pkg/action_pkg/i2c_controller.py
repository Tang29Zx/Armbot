#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from smbus2 import SMBus, i2c_msg
import struct

# ========== 根据实际情况修改这两个常量 ==========
I2C_BUS = 5                     # 你的 I2C 总线编号
I2C_SLAVE_ADDR = 0x30           # STM32 从机地址（7位）
# 如果你的 STM32 实际地址是 0x34，请改为 0x34
# ===============================================

CMD_PACKET_SIZE = 32
STATUS_SIZE = 8

class I2CMasterNode(Node):
    def __init__(self):
        super().__init__('i2c_master_node')

        # 打开 I2C 总线
        self.bus = SMBus(I2C_BUS)

        # 订阅命令话题
        self.sub = self.create_subscription(
            String,
            'command_topic',
            self.cmd_callback,
            10)

        # 发布状态话题
        self.status_pub = self.create_publisher(String, 'status_topic', 10)

        # 定时读取 STM32 状态（每 0.1 秒）
        self.timer = self.create_timer(0.1, self.read_status)

        self.get_logger().info(f'I2C Master Node started on bus {I2C_BUS}, addr 0x{I2C_SLAVE_ADDR:02X}')

    # ---------- I2C 底层操作（纯数据流，无寄存器地址） ----------
    def i2c_write_cmd(self, data_bytes):
        """发送 32 字节命令包到 STM32"""
        msg = i2c_msg.write(I2C_SLAVE_ADDR, list(data_bytes))
        self.bus.i2c_rdwr(msg)
        self.get_logger().debug(f'I2C write: {list(data_bytes)}')

    def i2c_read_status(self):
        """从 STM32 读取 8 字节状态包"""
        msg = i2c_msg.read(I2C_SLAVE_ADDR, STATUS_SIZE)
        self.bus.i2c_rdwr(msg)
        return list(msg)

    # ---------- 命令解析与 I2C 发送 ----------
    def cmd_callback(self, msg):
        """ROS 话题回调：解析字符串并构建 I2C 命令"""
        data = msg.data.strip()
        parts = data.split()
        if not parts:
            return

        cmd_type = parts[0].upper()
        cmd = None

        try:
            if cmd_type == 'ARM':
                cmd = self.build_arm_cmd(parts[1:])
            elif cmd_type == 'CAR':
                cmd = self.build_chassis_cmd(parts[1:])
            elif cmd_type == 'SERVO':
                cmd = self.build_servo_cmd(parts[1:])
            elif cmd_type == 'STOP':
                cmd = self.build_stop_cmd()
            else:
                self.get_logger().warn(f'Unknown command: {cmd_type}')
                return

            if cmd is not None:
                self.i2c_write_cmd(cmd)
                self.get_logger().info(f'Sent {cmd_type} command')
        except Exception as e:
            self.get_logger().error(f'Failed to send {cmd_type}: {e}')

    def read_status(self):
        """定时读取状态并发布到 /status_topic"""
        try:
            data = self.i2c_read_status()
            # 将字节转为字符串，过滤尾部空字符和填充
            status_str = bytes(data).decode('utf-8', errors='ignore').rstrip('\x00')
            msg = String()
            msg.data = status_str
            self.status_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Status read failed: {e}')

    # ---------- 命令包构建函数 ----------
    def build_arm_cmd(self, args):
        """构建 ARM 指令包：'A' + 7个浮点数 + uint32时间"""
        if len(args) < 7:
            self.get_logger().error('ARM needs 7 params (x y z pitch min_pitch max_pitch time)')
            return None

        x, y, z, pitch, min_pitch, max_pitch, time = map(float, args[:7])
        buf = bytearray(CMD_PACKET_SIZE)
        buf[0] = ord('A')            # 命令类型
        struct.pack_into('<f', buf, 4, x)
        struct.pack_into('<f', buf, 8, y)
        struct.pack_into('<f', buf, 12, z)
        struct.pack_into('<f', buf, 16, pitch)
        struct.pack_into('<f', buf, 20, min_pitch)
        struct.pack_into('<f', buf, 24, max_pitch)
        struct.pack_into('<I', buf, 28, int(time))
        return buf

    def build_chassis_cmd(self, args):
        """构建底盘指令包：'C' + uint16 angle + uint8 speed + int8 rot + bool drift"""
        if len(args) < 4:
            self.get_logger().error('CAR needs 4 params (angle speed rot drift)')
            return None

        angle = int(float(args[0])) % 360
        speed = max(0, min(100, int(float(args[1]))))
        rot = max(-100, min(100, int(float(args[2]))))
        drift = 1 if args[3].lower() in ('1', 'true') else 0

        buf = bytearray(CMD_PACKET_SIZE)
        buf[0] = ord('C')
        struct.pack_into('<H', buf, 4, angle)
        buf[6] = speed
        buf[7] = rot & 0xFF
        buf[8] = drift
        return buf

    def build_servo_cmd(self, args):
        """构建单舵机指令包：'P' + uint8 id + float angle"""
        if len(args) < 2:
            self.get_logger().error('SERVO needs id and angle')
            return None

        sid = max(1, min(6, int(float(args[0]))))
        angle = float(args[1])
        buf = bytearray(CMD_PACKET_SIZE)
        buf[0] = ord('P')
        buf[4] = sid
        struct.pack_into('<f', buf, 8, angle)
        return buf

    def build_stop_cmd(self):
        """构建停止指令包：'S'"""
        buf = bytearray(CMD_PACKET_SIZE)
        buf[0] = ord('S')
        return buf


def main(args=None):
    rclpy.init(args=args)
    node = I2CMasterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()