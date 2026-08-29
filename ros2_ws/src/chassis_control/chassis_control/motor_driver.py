#!/usr/bin/env python3
"""
motor_driver.py - Hiwonder 电机驱动板 (0x34) I2C 接口
对应 STM32 motor_module.c

依赖：pip install smbus2
"""

import struct
import time

try:
    import smbus2
except ImportError:
    print("请先安装 smbus2: pip install smbus2")
    raise

# ─── 寄存器定义（来自 motor_module.h）─────────────────────────
REG_BATTERY       = 0x00     # 电池电压 (uint16, mV)
REG_MOTOR_TYPE    = 0x14     # 电机类型
REG_POLARITY      = 0x15     # 编码器极性
REG_SET_PWM       = 0x1F     # 开环 PWM (4×int8, [-100,100])
REG_SET_SPEED     = 0x33     # 闭环速度 (4×int8, [-100,100])
REG_GET_ENCODER   = 0x3C     # 编码器计数 (4×int32)

MOTOR_TYPE_JGB37_520 = 3     # JGB37-520 12V 110RPM (520:1)


class MotorDriver:
    """与 Hiwonder 电机驱动板 (0x34) 的 I2C 通信封装"""

    def __init__(self, bus_num=5, addr=0x34):
        """
        Parameters
        ----------
        bus_num : int  I2C 总线号 (RDK X5 Pin3/Pin5 → /dev/i2c-5)
        addr    : int  设备 7-bit 地址 (0x34)
        """
        self.bus = smbus2.SMBus(bus_num)
        self.addr = addr

    # ─── 底层 I2C 读写 ─────────────────────────────────────

    def _write_reg(self, reg, data_bytes):
        """写寄存器：[reg, data0, data1, ...] (对应 HAL_I2C_Master_Transmit)
        8-29: 加重试——偶发 Errno 121（电机干扰/线束接触）重试 3 次，避免节点崩溃"""
        last_err = None
        for attempt in range(3):
            try:
                self.bus.write_i2c_block_data(self.addr, reg, data_bytes)
                return
            except OSError as e:
                last_err = e
                time.sleep(0.02)
        raise last_err

    def _read_reg(self, reg, length):
        """读寄存器 (对应 receive_from_device)
        8-29: 加重试——偶发 Errno 121 重试 3 次，避免节点崩溃"""
        last_err = None
        for attempt in range(3):
            try:
                return self.bus.read_i2c_block_data(self.addr, reg, length)
            except OSError as e:
                last_err = e
                time.sleep(0.02)
        raise last_err

    @staticmethod
    def _int8_to_uint8(val):
        """有符号 int8 → 无符号 uint8"""
        val = max(-100, min(100, int(val)))
        return val if val >= 0 else val + 256

    # ─── 初始化 ────────────────────────────────────────────

    def init(self, motor_type=MOTOR_TYPE_JGB37_520, polarity=0):
        """初始化电机类型和极性。第一步写停机，防止上电疯转"""
        # ⚠️ 安全第一：先停掉所有电机（清零速度和 PWM）
        # 无论 MCU 残留什么值，这一帧直接覆盖
        self._write_reg(REG_SET_SPEED, [0, 0, 0, 0])
        time.sleep(0.01)
        self._write_reg(REG_SET_PWM, [0, 0, 0, 0])
        time.sleep(0.01)

        # 然后再配电机类型和极性
        self._write_reg(REG_MOTOR_TYPE, [motor_type])
        time.sleep(0.05)
        self._write_reg(REG_POLARITY, [polarity])
        time.sleep(0.05)

    # ─── 紧急停机 ──────────────────────────────────────────

    def emergency_stop(self):
        """紧急停机：同时清零速度+PWM 寄存器，写 3 遍确保送达"""
        for _ in range(3):
            self._write_reg(REG_SET_SPEED, [0, 0, 0, 0])
            time.sleep(0.01)
            self._write_reg(REG_SET_PWM, [0, 0, 0, 0])
            time.sleep(0.01)

    # ─── 析构保护 ──────────────────────────────────────────

    def __del__(self):
        """对象销毁时尽力停车（不保证一定能执行，仅供参考）"""
        try:
            self.stop()
        except:
            pass

    # ─── 设置电机速度 ──────────────────────────────────────

    def set_speed(self, speeds):
        """设置 4 路电机闭环速度 (REG 0x33), 范围 [-100, 100]"""
        data = [self._int8_to_uint8(s) for s in speeds]
        self._write_reg(REG_SET_SPEED, data)

    # ─── 设置电机 PWM ──────────────────────────────────────

    def set_pwm(self, pwms):
        """设置 4 路电机开环 PWM (REG 0x1F), 范围 [-100, 100]"""
        data = [self._int8_to_uint8(p) for p in pwms]
        self._write_reg(REG_SET_PWM, data)

    # ─── 读取编码器 ────────────────────────────────────────

    def get_encoder(self):
        """读取 4 路编码器 (REG 0x3C), 返回 list of int32"""
        raw = self._read_reg(REG_GET_ENCODER, 16)
        return [struct.unpack('<i', bytes(raw[i*4:i*4+4]))[0] for i in range(4)]

    # ─── 读取电池电压 ─────────────────────────────────────

    def get_battery(self):
        """读取电池电压 (REG 0x00), 返回 mV"""
        raw = self._read_reg(REG_BATTERY, 2)
        return struct.unpack('<H', bytes(raw))[0]

    # ─── 停车 / 关闭 ──────────────────────────────────────

    def stop(self):
        """停车：同时清零速度和 PWM"""
        self._write_reg(REG_SET_SPEED, [0, 0, 0, 0])
        self._write_reg(REG_SET_PWM, [0, 0, 0, 0])

    def close(self):
        self.bus.close()
