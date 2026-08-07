"""RDK-local exclusive command mux and VLA heartbeat watchdog."""

import time

from action_interfaces.msg import ArmCommand, ArmState
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty
from std_srvs.srv import SetBool


class ArmCommandMuxNode(Node):
    def __init__(self, **kwargs):
        super().__init__("arm_command_mux_node", **kwargs)
        self._declare_parameters()
        self._vla_enabled = False
        self._teleop_enabled = False
        self._teleop_synced = False
        self._teleop_drop_warned = False
        self._latest_state = None
        self._state_received_at = None
        self._heartbeat_received_at = None
        self._last_forwarded_sequence = 0

        self._command_pub = self.create_publisher(
            ArmCommand, self._cfg("output_topic"), 10
        )
        enabled_qos = QoSProfile(depth=1)
        enabled_qos.reliability = ReliabilityPolicy.RELIABLE
        enabled_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._vla_enabled_pub = self.create_publisher(
            Bool, self._cfg("vla_enabled_topic"), enabled_qos
        )

        self._teleop_sub = self.create_subscription(
            ArmCommand,
            self._cfg("teleop_command_topic"),
            self._on_teleop_command,
            10,
        )
        self._vla_sub = self.create_subscription(
            ArmCommand,
            self._cfg("vla_command_topic"),
            self._on_vla_command,
            10,
        )
        heartbeat_qos = QoSProfile(depth=1)
        heartbeat_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self._heartbeat_sub = self.create_subscription(
            Empty,
            self._cfg("heartbeat_topic"),
            self._on_heartbeat,
            heartbeat_qos,
        )
        self._state_sub = self.create_subscription(
            ArmState, self._cfg("state_topic"), self._on_state, 10
        )
        self._teleop_enabled_sub = self.create_subscription(
            Bool,
            self._cfg("teleop_enabled_topic"),
            self._on_teleop_enabled,
            enabled_qos,
        )
        self._teleop_synced_sub = self.create_subscription(
            Bool,
            self._cfg("teleop_synced_topic"),
            self._on_teleop_synced,
            enabled_qos,
        )
        self._enable_service = self.create_service(
            SetBool, self._cfg("enable_service"), self._set_vla_enabled
        )
        self._watchdog_timer = self.create_timer(0.05, self._watchdog_tick)
        self._publish_vla_enabled()
        self.get_logger().info("arm command mux started; source=teleop")

    def _declare_parameters(self):
        defaults = {
            "output_topic": "/arm/command",
            "teleop_command_topic": "/arm/command/teleop",
            "vla_command_topic": "/arm/command/vla",
            "heartbeat_topic": "/vla/heartbeat",
            "state_topic": "/arm/state",
            "teleop_enabled_topic": "/arm/teleop_enabled",
            "teleop_synced_topic": "/arm/teleop_synced",
            "vla_enabled_topic": "/arm/vla_enabled",
            "enable_service": "/arm/set_vla_enabled",
            "heartbeat_timeout_sec": 0.30,
            "state_timeout_sec": 0.50,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _cfg(self, name):
        return self.get_parameter(name).value

    def _on_state(self, message):
        self._latest_state = message
        self._state_received_at = time.monotonic()

    def _on_heartbeat(self, _message):
        self._heartbeat_received_at = time.monotonic()

    def _on_teleop_enabled(self, message):
        self._teleop_enabled = bool(message.data)

    def _on_teleop_synced(self, message):
        self._teleop_synced = bool(message.data)
        if self._teleop_synced:
            self._teleop_drop_warned = False

    def _on_teleop_command(self, message):
        if self._vla_enabled:
            return
        if self._teleop_enabled and not self._teleop_synced:
            if not self._teleop_drop_warned:
                self.get_logger().error("dropping teleop motion until Home is verified")
                self._teleop_drop_warned = True
            return
        self._forward(message)

    def _on_vla_command(self, message):
        if not self._vla_enabled:
            return
        sequence = int(message.sequence_id)
        if sequence <= 0:
            self._disable_vla("VLA command has invalid sequence_id", send_stop=True)
            return
        if sequence <= self._last_forwarded_sequence:
            self._disable_vla("VLA command sequence is stale", send_stop=True)
            return
        if not self._runtime_health_reason(time.monotonic()):
            self._forward(message)

    def _forward(self, message):
        sequence = int(message.sequence_id)
        if sequence > 0:
            self._last_forwarded_sequence = max(self._last_forwarded_sequence, sequence)
        self._command_pub.publish(message)

    def _enable_reasons(self, now):
        reasons = []
        if self._teleop_enabled:
            reasons.append("disable Xbox teleop first")
        if not self._teleop_synced:
            reasons.append("Home has not been verified")
        heartbeat_age = self._age(now, self._heartbeat_received_at)
        if heartbeat_age > float(self._cfg("heartbeat_timeout_sec")):
            reasons.append("VLA heartbeat is stale")
        state_age = self._age(now, self._state_received_at)
        if state_age > float(self._cfg("state_timeout_sec")):
            reasons.append("ArmState is stale")
        state = self._latest_state
        if state is None:
            reasons.append("ArmState is missing")
        else:
            if not state.position_valid:
                reasons.append("joint feedback is invalid")
            if state.error_code != 0:
                reasons.append("arm error_code=%d" % state.error_code)
            if state.state not in (ArmState.STATE_IDLE, ArmState.STATE_SUCCEEDED):
                reasons.append("arm must be idle before VLA acquisition")
        return reasons

    def _runtime_health_reason(self, now):
        if self._teleop_enabled:
            return "Xbox teleop became enabled"
        if self._age(now, self._heartbeat_received_at) > float(
            self._cfg("heartbeat_timeout_sec")
        ):
            return "VLA heartbeat timeout"
        if self._age(now, self._state_received_at) > float(
            self._cfg("state_timeout_sec")
        ):
            return "ArmState timeout"
        state = self._latest_state
        if state is None:
            return "ArmState missing"
        if not state.position_valid:
            return "joint feedback became invalid"
        if state.state in (ArmState.STATE_ERROR, ArmState.STATE_ESTOP):
            return "arm entered ERROR/ESTOP"
        if state.error_code != 0:
            return "arm error_code=%d" % state.error_code
        return ""

    @staticmethod
    def _age(now, timestamp):
        return float("inf") if timestamp is None else now - timestamp

    def _set_vla_enabled(self, request, response):
        if request.data:
            if self._vla_enabled:
                response.success = True
                response.message = "VLA control is already enabled"
                return response
            reasons = self._enable_reasons(time.monotonic())
            if reasons:
                response.success = False
                response.message = "; ".join(reasons)
                return response
            self._vla_enabled = True
            # Own the post-VLA Home interlock locally as well as notifying
            # teleop. This closes the short enable/disable race in which the
            # transient ownership update has not reached teleop yet.
            self._teleop_synced = False
            self._teleop_drop_warned = False
            self._publish_vla_enabled()
            response.success = True
            response.message = "VLA control enabled"
            self.get_logger().warn(response.message)
            return response

        if self._vla_enabled:
            self._disable_vla("operator request", send_stop=True)
        response.success = True
        response.message = "VLA control disabled; Home required for teleop"
        return response

    def _watchdog_tick(self):
        if not self._vla_enabled:
            return
        reason = self._runtime_health_reason(time.monotonic())
        if reason:
            self._disable_vla(reason, send_stop=True)

    def _disable_vla(self, reason, send_stop):
        self._vla_enabled = False
        self._publish_vla_enabled()
        if send_stop:
            self._publish_stop()
        self.get_logger().error("VLA disabled: %s" % reason)

    def _publish_stop(self):
        state_sequence = (
            int(self._latest_state.sequence_id) if self._latest_state is not None else 0
        )
        sequence = max(self._last_forwarded_sequence, state_sequence) + 1
        sequence &= 0xFFFFFFFF
        if sequence == 0:
            sequence = 1
        message = ArmCommand()
        message.mode = ArmCommand.MODE_STOP
        message.sequence_id = sequence
        self._last_forwarded_sequence = sequence
        self._command_pub.publish(message)

    def _publish_vla_enabled(self):
        self._vla_enabled_pub.publish(Bool(data=self._vla_enabled))

    def destroy_node(self):
        if self._vla_enabled:
            self._publish_stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmCommandMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
