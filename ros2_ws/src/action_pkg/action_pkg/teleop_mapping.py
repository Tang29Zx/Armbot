"""Pure Xbox-to-arm target mapping with no ROS or hardware dependencies."""

from dataclasses import dataclass, replace
import math


MODE_ARM = 'arm'
MODE_GRIPPER = 'gripper'


@dataclass(frozen=True)
class Target:
    """Maintained absolute arm target and normalized gripper target."""

    x: float = 15.0
    y: float = 0.0
    z: float = 2.0
    pitch: float = -54.48
    gripper: float = 0.0


def clamp(value, lower, upper):
    """Clamp a scalar to an inclusive range."""
    return max(lower, min(upper, value))


def apply_deadzone(value, threshold):
    """Remove stick drift and rescale the remaining travel to [-1, 1]."""
    if not math.isfinite(value) or not 0.0 <= threshold < 1.0:
        raise ValueError('invalid axis value or deadzone')
    magnitude = abs(value)
    if magnitude <= threshold:
        return 0.0
    scaled = (magnitude - threshold) / (1.0 - threshold)
    return math.copysign(scaled, value)


def trigger_pressed(raw):
    """Convert Xbox trigger raw range (+1 released, -1 pressed) to [0, 1]."""
    if not math.isfinite(raw):
        raise ValueError('non-finite trigger value')
    return clamp((1.0 - raw) / 2.0, 0.0, 1.0)


def valid_joy(axes, buttons):
    """Check the required Xbox axes/buttons before indexing them."""
    return (
        len(axes) >= 6
        and len(buttons) >= 8
        and all(math.isfinite(float(value)) for value in axes[:6])
    )


def rising_edge(buttons, previous_buttons, index):
    """Return true once when a button changes from released to pressed."""
    current = index < len(buttons) and bool(buttons[index])
    previous = index < len(previous_buttons) and bool(previous_buttons[index])
    return current and not previous


def controls_neutral(axes, deadzone, trigger_deadzone=0.05):
    """Check that enabling cannot immediately create a motion command."""
    if len(axes) < 6:
        return False
    sticks = (axes[0], axes[1], axes[2], axes[3])
    return (
        all(apply_deadzone(float(value), deadzone) == 0.0 for value in sticks)
        and trigger_pressed(float(axes[4])) <= trigger_deadzone
        and trigger_pressed(float(axes[5])) <= trigger_deadzone
    )


def integrate_target(target, axes, dt, *, deadzone, translation_speed,
                     pitch_speed, gripper_speed, bounds,
                     trigger_deadzone=0.05):
    """Integrate a tick, giving Cartesian input priority over gripper."""
    if len(axes) < 6 or not math.isfinite(dt) or dt <= 0.0:
        return target, None

    x_axis = apply_deadzone(float(axes[1]), deadzone)
    y_axis = apply_deadzone(float(axes[0]), deadzone)
    z_axis = apply_deadzone(float(axes[3]), deadzone)
    pitch_axis = apply_deadzone(float(axes[2]), deadzone)

    if any(value != 0.0 for value in (x_axis, y_axis, z_axis, pitch_axis)):
        updated = replace(
            target,
            x=target.x + x_axis * translation_speed * dt,
            y=target.y + y_axis * translation_speed * dt,
            z=target.z + z_axis * translation_speed * dt,
            pitch=clamp(target.pitch + pitch_axis * pitch_speed * dt,
                        *bounds['pitch']),
        )
        return updated, MODE_ARM

    close_amount = trigger_pressed(float(axes[4]))
    open_amount = trigger_pressed(float(axes[5]))
    gripper_axis = close_amount - open_amount
    if abs(gripper_axis) <= trigger_deadzone:
        return target, None
    updated = replace(
        target,
        gripper=clamp(target.gripper + gripper_axis * gripper_speed * dt,
                      0.0, 1.0),
    )
    return updated, MODE_GRIPPER


def joints_near_home(actual, expected, tolerance_rad):
    """Verify all measured joints are finite and near the known reset pose."""
    return (
        len(actual) == len(expected)
        and all(math.isfinite(value) for value in actual)
        and all(abs(value - wanted) <= tolerance_rad
                for value, wanted in zip(actual, expected))
    )
