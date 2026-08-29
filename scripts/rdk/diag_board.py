#!/usr/bin/env python3
"""驱动板寄存器诊断：读电压/编码器/电机类型/极性，判断驱动板是否正常
用法: python3 /tmp/diag_board.py
"""
import smbus2
import struct
import time

bus = smbus2.SMBus(5)
addr = 0x34

REG_BATTERY    = 0x00
REG_MOTOR_TYPE = 0x14
REG_POLARITY   = 0x15
REG_SET_PWM    = 0x1F
REG_SET_SPEED  = 0x33
REG_GET_ENCODER = 0x3C


def read_reg(reg, length):
    raw = bus.read_i2c_block_data(addr, reg, length)
    return raw


print("=== 驱动板寄存器诊断 (I2C5 / 0x34) ===", flush=True)
try:
    bat = read_reg(REG_BATTERY, 2)
    volt = struct.unpack('<H', bytes(bat))[0]
    print(f"电池电压: {volt/1000.0:.2f} V", flush=True)
except Exception as e:
    print(f"读电压失败: {e}", flush=True)

try:
    mtype = read_reg(REG_MOTOR_TYPE, 1)
    pol = read_reg(REG_POLARITY, 1)
    print(f"电机类型: {mtype[0]} (3=JGB37-520), 极性: {pol[0]}", flush=True)
except Exception as e:
    print(f"读电机类型失败: {e}", flush=True)

try:
    enc1 = read_reg(REG_GET_ENCODER, 16)
    enc = [struct.unpack('<i', bytes(enc1[i*4:i*4+4]))[0] for i in range(4)]
    print(f"编码器: {enc}", flush=True)
except Exception as e:
    print(f"读编码器失败: {e}", flush=True)

print("=== 尝试写入速度 30 并读回编码器 2 秒（看轮子是否转） ===", flush=True)
try:
    bus.write_i2c_block_data(addr, REG_SET_SPEED, [30, 30, 30, 30])
    t0 = time.time()
    e0 = enc
    while time.time() - t0 < 2.0:
        time.sleep(0.5)
        e1_raw = read_reg(REG_GET_ENCODER, 16)
        e1 = [struct.unpack('<i', bytes(e1_raw[i*4:i*4+4]))[0] for i in range(4)]
        print(f"  enc: {e1}", flush=True)
    bus.write_i2c_block_data(addr, REG_SET_SPEED, [0, 0, 0, 0])
    print("写入速度 30 后编码器变化: ", [e1[i]-e0[i] for i in range(4)], flush=True)
except Exception as e:
    print(f"速度测试失败: {e}", flush=True)
print("=== 诊断完成 ===", flush=True)
