#!/usr/bin/env python3
"""
Gripper travel-range probe that talks to I2C directly.

Stops the ROS node first (only one I2C master may own /dev/i2c-5 at a time).
Sends SERVO commands (tag 'P', id=1) with a sweep of raw values and reads
back the gripper servo's *actual* position from the firmware status packet,
so we can see exactly which raw values the gripper responds to and what its
true closed/open extremes are.

Firmware contract (verified in Core/Src/main.c + Hiwonder serial_servo.c):
  SERVO cmd: buf[0]='P', buf[4]=servo_id, buf[8..11]=float32 raw_angle
  -> HostServoSet(id, raw) -> serial_servo_set_position(controller, id, raw, 1000)
  status packet: 8-byte ASCII status + 6 x float32 servo raw positions (byte 8..31)

Run on RDK (after Ctrl-C the arm_controller_node):
  source /opt/ros/humble/setup.bash
  source ~/Armbot/ros2_ws/install/setup.bash
  python3 ~/Armbot/ros2_ws/src/action_pkg/action_pkg/gripper_probe.py
"""
import struct
import time

from smbus2 import i2c_msg, SMBus


def read_status(bus):
    """Read 32-byte status packet; return (status_str, 6 servo raw floats)."""
    for _ in range(5):
        try:
            r = i2c_msg.read(0x30, 32)
            bus.i2c_rdwr(r)
            data = list(r)
            status = bytes(data[:8]).decode('utf-8', errors='ignore').rstrip('\x00').strip()
            pos = struct.unpack('<6f', bytes(data[8:32]))
            return status, pos
        except OSError:
            time.sleep(0.05)
    return 'NAK', [0.0] * 6


def write_cmd(bus, buf):
    for _ in range(5):
        try:
            w = i2c_msg.write(0x30, list(buf))
            bus.i2c_rdwr(w)
            return True
        except OSError:
            time.sleep(0.05)
    return False


def build_servo(sid=1, raw=0.0):
    buf = bytearray(32)
    buf[0] = ord('P')
    buf[4] = sid & 0xFF
    struct.pack_into('<f', buf, 8, raw)
    return buf


def main():
    bus = SMBus(5)
    print('[probe] I2C opened ok (bus=5 addr=0x30)')

    # baseline idle read
    s, pos = read_status(bus)
    print('[probe] idle status=%r servo1_raw=%.1f (servo ids: 1..6 = %s)' % (
        s, pos[0], ['%.0f' % p for p in pos]))

    sweep = [0.0, 250.0, 400.0, 500.0, 600.0, 750.0, 1000.0]
    print('\n[probe] --- gripper raw sweep (cmd raw -> actual servo1_raw) ---')
    results = []
    for raw in sweep:
        write_cmd(bus, build_servo(1, raw))
        time.sleep(1.6)  # give the servo time to move
        s, pos = read_status(bus)
        actual = pos[0]
        moved = abs(actual - (results[-1][1] if results else pos[0])) > 1.0
        print('  cmd raw=%6.0f -> servo1_raw=%7.1f   status=%r   %s' % (
            raw, actual, s, '<< moved' if moved else ''))
        results.append((raw, actual))

    print('\n=== SUMMARY (cmd_raw -> actual_servo1_raw) ===')
    for raw, actual in results:
        print('  %6.0f -> %7.1f' % (raw, actual))
    print('\n[probe] interpretation:')
    print('  - the gripper travels between the min and max actual_servo1_raw above.')
    print('  - set gripper_closed_raw = the raw that gives the *closed* extreme,')
    print('    gripper_open_raw  = the raw that gives the *open*  extreme.')
    print('[probe] done.')


if __name__ == '__main__':
    main()
