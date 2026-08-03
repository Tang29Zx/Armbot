#!/usr/bin/env python3
"""Publish a causally filtered ArmState for VLA observations."""

from copy import deepcopy
import math

from action_interfaces.msg import ArmState
from action_pkg.state_filter import MedianOneEuroFilter
import rclpy
from rclpy.node import Node


FILTER_DIMENSIONS = 6
JOINT_COUNT = 5


class ArmStateFilterNode(Node):
    """Keep raw control state untouched and publish a filtered derivative."""

    def __init__(self):
        super().__init__('arm_state_filter_node')
        self.declare_parameter('input_topic', '/arm/state')
        self.declare_parameter('output_topic', '/arm/state_filtered')
        self.declare_parameter('state_filter_window_size', 3)
        self.declare_parameter('state_filter_min_cutoff_hz', 1.0)
        self.declare_parameter('state_filter_beta', 1.5)
        self.declare_parameter('state_filter_derivative_cutoff_hz', 1.0)
        self.declare_parameter('state_filter_reset_gap_sec', 0.5)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        window_size = int(
            self.get_parameter('state_filter_window_size').value)
        min_cutoff_hz = float(
            self.get_parameter('state_filter_min_cutoff_hz').value)
        beta = float(self.get_parameter('state_filter_beta').value)
        derivative_cutoff_hz = float(
            self.get_parameter('state_filter_derivative_cutoff_hz').value)
        reset_gap_sec = float(
            self.get_parameter('state_filter_reset_gap_sec').value)
        self._filter = MedianOneEuroFilter(
            FILTER_DIMENSIONS,
            window_size=window_size,
            min_cutoff_hz=min_cutoff_hz,
            beta=beta,
            derivative_cutoff_hz=derivative_cutoff_hz,
            reset_gap_sec=reset_gap_sec,
        )
        self._publisher = self.create_publisher(
            ArmState, output_topic, 10)
        self._subscription = self.create_subscription(
            ArmState, input_topic, self._state_callback, 10)
        self.get_logger().info(
            ('filtering %s -> %s with median window=%d, One Euro '
             'min_cutoff=%.3f Hz, beta=%.3f, d_cutoff=%.3f Hz')
            % (input_topic, output_topic, window_size, min_cutoff_hz,
               beta, derivative_cutoff_hz))

    def _state_callback(self, msg):
        output = deepcopy(msg)
        values = list(msg.joint_position) + [msg.gripper_position]
        healthy = (
            msg.position_valid
            and len(msg.joint_position) == JOINT_COUNT
            and msg.state not in (ArmState.STATE_ERROR, ArmState.STATE_ESTOP)
            and all(math.isfinite(value) for value in values)
        )
        if not healthy:
            self._filter.reset()
            output.position_valid = False
            self._publisher.publish(output)
            return

        timestamp_sec = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1e-9
        )
        filtered = self._filter.update(values, timestamp_sec)
        output.joint_position = filtered[:JOINT_COUNT]
        output.gripper_position = filtered[-1]
        self._publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = ArmStateFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
