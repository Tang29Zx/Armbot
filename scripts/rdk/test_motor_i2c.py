#!/usr/bin/env python3
"""单电机通道测试：开环 PWM 逐通道驱动，观察哪个轮子转
用法: python3 /tmp/test_motor_i2c.py  （需 root/sunrise 有 I2C 权限，底盘节点须已停止）
"""
import smbus2
import time

bus = smbus2.SMBus(5)
addr = 0x34
REG_PWM = 0x1F
REG_SPEED = 0x33


def stop_all():
    bus.write_i2c_block_data(addr, REG_SPEED, [0, 0, 0, 0])
    time.sleep(0.05)
    bus.write_i2c_block_data(addr, REG_PWM, [0, 0, 0, 0])
    time.sleep(0.2)


names = ["M1右前(通道1)", "M2右后(通道2)", "M3左前(通道3)", "M4左后(通道4)"]
stop_all()
print("=== 逐通道开环测试 (PWM=60, 每通道1.5s) ===", flush=True)
for i, name in enumerate(names):
    pwm = [0, 0, 0, 0]
    pwm[i] = 60
    print(f">>> 驱动 {name}: PWM={pwm}（观察该轮是否转）", flush=True)
    bus.write_i2c_block_data(addr, REG_PWM, pwm)
    time.sleep(1.5)
    bus.write_i2c_block_data(addr, REG_PWM, [0, 0, 0, 0])
    time.sleep(0.8)

print(">>> 反向测试: 所有轮 PWM=60 前进方向", flush=True)
bus.write_i2c_block_data(addr, REG_PWM, [60, 60, 60, 60])
time.sleep(1.5)
stop_all()
print("=== 测试完成，请记录每个通道轮子是否转动 ===", flush=True)
