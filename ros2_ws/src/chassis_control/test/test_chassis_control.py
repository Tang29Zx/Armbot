import math

import pytest
from geometry_msgs.msg import Twist

from chassis_control.chassis_control_node import mecanum_inverse, twist_to_motor_speeds
from chassis_control.odometry import ENCODER_PPR, Odometry, _norm_angle


class _FakeMotorDriver:
    def __init__(self, samples: list[list[int]]) -> None:
        self._samples = iter(samples)

    def get_encoder(self) -> list[int]:
        return next(self._samples)


def test_mecanum_inverse_clamps_motor_commands() -> None:
    assert mecanum_inverse(90.0, 200.0, 200.0) == [100, -100, -41, 41]


def test_forward_twist_produces_forward_wheel_pattern() -> None:
    twist = Twist()
    twist.linear.x = 0.5

    speeds, angle, speed, rot = twist_to_motor_speeds(twist, 0.5, 2.0)

    assert speeds == [70, -70, -70, 70]
    assert angle == pytest.approx(90.0)
    assert speed == pytest.approx(100.0)
    assert rot == pytest.approx(0.0)


def test_odometry_integrates_forward_encoder_motion() -> None:
    counts = ENCODER_PPR // 10
    driver = _FakeMotorDriver([
        [0, 0, 0, 0],
        [counts, -counts, -counts, counts],
    ])
    odometry = Odometry(driver, sample_dt=0.1)

    odometry.init()
    updated = odometry.update()

    assert updated is True
    assert odometry.pose.x > 0.0
    assert odometry.pose.y == pytest.approx(0.0)
    assert odometry.pose.theta == pytest.approx(0.0)


def test_norm_angle_wraps_to_pi_range() -> None:
    assert _norm_angle(3.0 * math.pi) == pytest.approx(math.pi)
