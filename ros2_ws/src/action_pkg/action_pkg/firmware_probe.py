#!/usr/bin/env python3
"""
Standalone firmware probe that bypasses the ROS node entirely.

Purpose: prove whether the firmware currently running on the STM32 actually
processes I2C ARM/STOP commands, or whether the burned binary is stale / the
firmware I2C slave stops responding after a write.

Run on the RDK with the arm_controller_node STOPPED (only one I2C master
process may talk to the slave at a time):

    source /opt/ros/humble/setup.bash
    source ~/Armbot/ros2_ws/install/setup.bash
    python3 ~/Armbot/ros2_ws/src/action_pkg/action_pkg/firmware_probe.py

Reads after a write often get a transient Remote I/O (Errno 121) because the
firmware is busy re-arming its I2C transmit buffer. This probe SLEEPS after
each write and RETRIES on Remote I/O, so it surfaces the real status the
firmware eventually reports.
"""
import struct
import time

try:
    from smbus2 import SMBus, i2c_msg
except ImportError:
    print('smbus2 not available; install: pip install smbus2')
    raise SystemExit(1)

BUS = 5
ADDR = 0x30
PROTOCOL_VERSION = 3
PROTOCOL_MAGIC = 0xA5


def status_str(data):
    raw = bytes(data)
    if len(raw) != 32 or raw[0] != PROTOCOL_MAGIC:
        return 'INVALID:%s' % raw[:8].hex()
    wire_id = struct.unpack_from('<I', raw, 4)[0]
    return 'v%d lifecycle=%d error=%d wire=%d' % (
        raw[1], raw[2], raw[3], wire_id)


def read_status(bus, retries=20, delay=0.02):
    """Read 32 bytes. Retry on transient Remote I/O (Errno 121)."""
    last_exc = None
    for _ in range(retries):
        try:
            r = i2c_msg.read(ADDR, 32)
            bus.i2c_rdwr(r)
            return list(r), None
        except OSError as e:
            last_exc = e
            if getattr(e, 'errno', None) == 121:
                time.sleep(delay)
                continue
            raise
    return None, last_exc


def write_cmd(bus, buf):
    w = i2c_msg.write(ADDR, list(buf))
    bus.i2c_rdwr(w)


def build_arm(x=20.0, y=0.0, z=15.0, pitch=0.0,
              min_pitch=-90.0, max_pitch=90.0, dur_ms=2000,
              wire_id=2):
    buf = bytearray(32)
    buf[0] = ord('A')
    buf[1] = PROTOCOL_VERSION
    struct.pack_into('<I', buf, 2, wire_id)
    struct.pack_into('<H', buf, 6, dur_ms)
    struct.pack_into('<f', buf, 8, x)
    struct.pack_into('<f', buf, 12, y)
    struct.pack_into('<f', buf, 16, z)
    struct.pack_into('<f', buf, 20, pitch)
    # min/max pitch form the IK roll window (set_pitch_range). Must be a
    # non-degenerate range; [0,0] over-constrains the IK -> NO_SOLVE.
    struct.pack_into('<f', buf, 24, min_pitch)
    struct.pack_into('<f', buf, 28, max_pitch)
    return buf


def watch(bus, label, duration, settle=0.15, nok=1):
    """
    Watch status for ``duration`` seconds after a caller write.

    Return the distinct status strings seen, excluding NAK responses.
    """
    print('\n[probe] --- %s (watching %.0fs, %.2fs settle) ---' % (label, duration, settle))
    time.sleep(settle)
    seen = {}
    t0 = time.time()
    naks = 0
    while time.time() - t0 < duration:
        d, err = read_status(bus)
        if d is None:
            naks += 1
            if naks <= nok or naks % 10 == 0:
                print('  [NAK #%d: %s]' % (naks, err))
            time.sleep(0.1)
            continue
        s = status_str(d)
        raw = struct.unpack_from('<f', bytes(d), 8)[0]
        if s not in seen:
            seen[s] = 0
            print('  -> status=%r servo1_raw=%.1f' % (s, raw))
        seen[s] += 1
        time.sleep(0.1)
    return seen


def main():
    bus = SMBus(BUS)
    print('[probe] I2C opened ok (bus=%d addr=0x%02X)' % (BUS, ADDR))

    # --- baseline: idle status ---
    print('\n[probe] --- idle reads (expect v3 READY) ---')
    idle = {}
    for _ in range(3):
        d, _ = read_status(bus)
        s = status_str(d)
        print('  status=%r' % s)
        idle[s] = idle.get(s, 0) + 1
        time.sleep(0.3)
    print('  idle seen:', idle)

    # --- STOP: establishes whether ANY command is processed ---
    stop = bytearray(32)
    stop[0] = ord('S')
    stop[1] = PROTOCOL_VERSION
    struct.pack_into('<I', stop, 2, 1)
    write_cmd(bus, stop)
    print('[probe] wrote STOP')
    stop_seen = watch(bus, 'after STOP', 3.0)

    # --- ARM: the real test ---
    write_cmd(bus, build_arm())
    print('[probe] wrote ARM (x=20 y=0 z=15 dur=2000ms)')
    arm_seen = watch(bus, 'after ARM', 8.0)

    bus.close()
    print('\n[probe] === SUMMARY ===')
    print('  idle  statuses:', idle)
    print('  STOP  statuses:', stop_seen)
    print('  ARM   statuses:', arm_seen)
    print('[probe] done.')


if __name__ == '__main__':
    main()
