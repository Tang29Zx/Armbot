#!/usr/bin/env python3
"""
Bare-metal YDLIDAR → /scan publisher (timer-based, optimised).

Optimizations over the original:
  1. DEG2RAD pre-computed (no numpy per-point)
  2. Manual byte unpacking (no struct.unpack_from overhead)
  3. Angle wrap-around handling for 0/360 crossing
  4. Running angle min/max tracking (O(1) per packet)
  5. Sample-rate config parameter
  6. Anti-corruption: discard packets with lsn > 200
"""

import math
import time
import array

import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


# ── Constants ──────────────────────────────────────────────────────────
PACKET_HEADER = bytes([0xAA, 0x55])
CMD_STOP  = bytes([0xA5, 0x65])
CMD_SCAN  = bytes([0xA5, 0x60])

DEG2RAD = math.pi / 180.0
FULL_CIRCLE_RAD = 2.0 * math.pi


class YDLidarRawNode(Node):
    """Optimised YDLIDAR scan publisher."""

    def __init__(self):
        super().__init__('ydlidar_raw_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 230400)
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('motor_hz', 8.0)
        self.declare_parameter('samp_rate', 4)
        self.declare_parameter('dump_scan', False)

        self.port      = self.get_parameter('port').value
        self.baud      = self.get_parameter('baudrate').value
        self.frame_id  = self.get_parameter('frame_id').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.motor_hz  = self.get_parameter('motor_hz').value
        self.samp_rate = self.get_parameter('samp_rate').value

        # ── Serial ────────────────────────────────────────────────────
        self.get_logger().info(f'Opening {self.port} @ {self.baud} baud')
        self.ser = serial.Serial(self.port, self.baud, timeout=0.5)
        self.ser.reset_input_buffer()

        # ── Publisher ──────────────────────────────────────────────────
        self.scan_pub = self.create_publisher(
            LaserScan, '/scan', rclpy.qos.qos_profile_sensor_data
        )

        # ── State ─────────────────────────────────────────────────────
        self.packet_buf = bytearray()
        self.scan_data  = array.array('d')  # interleaved [ang, rng, ...]
        self.angle_min  = 999.0
        self.angle_max  = -999.0
        self._seen_large_angle = False  # for wrap-around detection

        # ── Start motor ────────────────────────────────────────────────
        self._start_motor()

        # ── Timer ──────────────────────────────────────────────────────
        self.timer = self.create_timer(1.0 / 100.0, self._read_loop)  # 100Hz

        self.get_logger().info(
            f'Node ready, motor={self.motor_hz}Hz samp={self.samp_rate}'
        )

    # ── Motor ──────────────────────────────────────────────────────────
    def _start_motor(self):
        self.ser.write(CMD_STOP)
        time.sleep(0.15)
        self.ser.reset_input_buffer()
        freq = int(max(3, min(15, self.motor_hz)))
        sr   = int(max(3, min(20, self.samp_rate)))
        self.ser.write(bytes([0xA5, 0x09, freq]))
        time.sleep(0.05)
        self.ser.write(bytes([0xA5, 0xD0, sr]))
        time.sleep(0.05)
        self.ser.reset_input_buffer()
        self.ser.write(CMD_SCAN)
        self.get_logger().info(f'SCAN @ {freq}Hz')

    def _stop_motor(self):
        self.ser.write(bytes([0xA5, 0x00]))
        time.sleep(0.03)
        self.ser.write(CMD_STOP)

    # ── Read loop (100Hz timer) ───────────────────────────────────────
    def _read_loop(self):
        try:
            n = self.ser.in_waiting
            if n > 0:
                self.packet_buf.extend(self.ser.read(n))

            while len(self.packet_buf) >= 10:
                idx = self.packet_buf.find(PACKET_HEADER)
                if idx < 0:
                    self.packet_buf.clear()
                    break
                if idx > 0:
                    del self.packet_buf[:idx]

                if len(self.packet_buf) < 10:
                    break

                lsn = self.packet_buf[3]
                if lsn > 200:  # corrupt, skip header
                    del self.packet_buf[:2]
                    break

                packet_len = 10 + lsn * 2
                if len(self.packet_buf) < packet_len:
                    break

                # Manual byte unpack (faster than struct)
                fsa = self.packet_buf[4] | (self.packet_buf[5] << 8)
                lsa = self.packet_buf[6] | (self.packet_buf[7] << 8)
                fsa_deg = fsa * 0.01
                lsa_deg = lsa * 0.01
                step = (lsa_deg - fsa_deg) / (lsn - 1) if lsn > 1 else 0.0

                for i in range(lsn):
                    off = 10 + i * 2
                    dist = self.packet_buf[off] | (self.packet_buf[off + 1] << 8)
                    rng = dist * 0.001  # mm → m
                    if self.range_min <= rng <= self.range_max:
                        ang = (fsa_deg + i * step) * DEG2RAD
                        # Wrap-around: add 2π when crossing 360°/0°
                        if not self._seen_large_angle and ang > 5.2:
                            self._seen_large_angle = True
                        if self._seen_large_angle and ang < 1.0:
                            ang += FULL_CIRCLE_RAD
                        self.scan_data.append(ang)
                        self.scan_data.append(rng)
                        if ang < self.angle_min:
                            self.angle_min = ang
                        if ang > self.angle_max:
                            self.angle_max = ang

                # Full revolution?
                if self.angle_max - self.angle_min > FULL_CIRCLE_RAD:
                    self._publish_scan()

                del self.packet_buf[:packet_len]

        except Exception as e:
            self.get_logger().error(f'Read: {e}', throttle_duration_sec=2.0)

    # ── Publish ───────────────────────────────────────────────────────
    def _publish_scan(self):
        if len(self.scan_data) < 4:
            return

        raw = self.scan_data[:]
        amin = self.angle_min
        amax = self.angle_max
        self.scan_data = array.array('d')
        self.angle_min  = 999.0
        self.angle_max  = -999.0
        # NOTE: _seen_large_angle is NOT reset — it stays True
        # across scans to handle 0°/360° wrap-around correctly.

        n = len(raw) // 2
        order = sorted(range(n), key=lambda i: raw[i * 2])

        msg = LaserScan()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.range_min       = self.range_min
        msg.range_max       = self.range_max
        msg.angle_min       = amin
        msg.angle_max       = amax
        msg.angle_increment = (amax - amin) / (n - 1) if n > 1 else 0.0049
        msg.ranges          = [raw[i * 2 + 1] for i in order]

        self.scan_pub.publish(msg)

        if n > 20 and self.get_parameter('dump_scan').value:
            import json, os
            msg.angle_min = float(amin)
            msg.angle_max = float(amax)
            dump = {
                'angle_min': float(amin), 'angle_max': float(amax),
                'angle_increment': float(msg.angle_increment),
                'ranges': [float(x) for x in msg.ranges], 'num_points': n,
            }
            out = os.path.expanduser('~/latest_scan.json')
            tmp = out + '.tmp'
            with open(tmp, 'w') as f:
                json.dump(dump, f)
            os.rename(tmp, out)  # atomic

        self.get_logger().info(
            f'Scan: {n} pts, [{amin:.2f},{amax:.2f}]',
            throttle_duration_sec=1.0,
        )

    def destroy_node(self):
        self._stop_motor()
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YDLidarRawNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == '__main__':
    main()
