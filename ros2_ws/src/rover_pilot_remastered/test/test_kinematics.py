import math

import pytest

from rover_pilot_remastered.kinematics import DriveGeometry, twist_to_wheel_rpm

ROVER = DriveGeometry(wheel_diameter_m=0.28, track_width_m=0.84, max_wheel_rpm=94.7)


def test_top_speed_is_5_kmh():
    assert ROVER.max_linear_speed == pytest.approx(5 / 3.6, rel=1e-3)
    assert ROVER.max_angular_speed == pytest.approx(2 * ROVER.max_linear_speed / 0.84)


def test_straight_and_spin():
    assert twist_to_wheel_rpm(ROVER.max_linear_speed, 0.0, ROVER) == pytest.approx((94.7,) * 4)
    fl, fr, rl, rr = twist_to_wheel_rpm(0.0, ROVER.max_angular_speed, ROVER)  # +yaw = turn left (CCW)
    assert (fl, fr) == pytest.approx((-94.7, 94.7)) and (rl, rr) == (fl, fr)


def test_saturation_keeps_the_turning_radius():
    fl, fr, _, _ = twist_to_wheel_rpm(ROVER.max_linear_speed, ROVER.max_angular_speed, ROVER)
    assert fl == pytest.approx(0.0, abs=1e-9) and fr == pytest.approx(94.7)
    fl, fr, _, _ = twist_to_wheel_rpm(-2 * ROVER.max_linear_speed, -0.5, ROVER)
    assert max(abs(fl), abs(fr)) == pytest.approx(94.7)
    left_speed, right_speed = -2 * ROVER.max_linear_speed + 0.5 * 0.42, -2 * ROVER.max_linear_speed - 0.5 * 0.42
    assert fl / fr == pytest.approx(left_speed / right_speed)


def test_other_wheel_size_changes_rpm_not_speed():
    big = DriveGeometry(wheel_diameter_m=0.35, track_width_m=0.84, max_wheel_rpm=94.7)
    fl, *_ = twist_to_wheel_rpm(1.0, 0.0, big)
    assert fl == pytest.approx(1.0 / (math.pi * 0.35) * 60)


def test_bad_inputs():
    assert twist_to_wheel_rpm(float('nan'), 0.0, ROVER) == (0.0, 0.0, 0.0, 0.0)
    with pytest.raises(ValueError):
        DriveGeometry(wheel_diameter_m=0.0, track_width_m=0.84, max_wheel_rpm=94.7)
