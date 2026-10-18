"""Skid-steer kinematics: /cmd_vel (REP 103: x forward m/s, +z yaw = counter-clockwise rad/s) -> wheel rpm."""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class DriveGeometry:
    wheel_diameter_m: float  # depends on the fitted wheel type (config/wheels/*.yaml)
    track_width_m: float  # distance between left and right wheel centres
    max_wheel_rpm: float  # wheel rpm at full PWM (open loop), a motor property
    turn_track_factor: float = 1.0  # > 1 compensates for skid-steer slip when turning

    def __post_init__(self) -> None:
        for name in ('wheel_diameter_m', 'track_width_m', 'max_wheel_rpm', 'turn_track_factor'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be a positive number, got {value}')

    @property
    def max_linear_speed(self) -> float:
        """Top speed in m/s (all wheels at max rpm)."""
        return self.max_wheel_rpm * math.pi * self.wheel_diameter_m / 60.0

    @property
    def max_angular_speed(self) -> float:
        """Spin-in-place rate in rad/s with both sides at max rpm in opposite directions."""
        return 2.0 * self.max_linear_speed / (self.track_width_m * self.turn_track_factor)


def twist_to_wheel_rpm(linear_x: float, angular_z: float, geometry: DriveGeometry) -> tuple[float, float, float, float]:
    """Returns wheel targets (FL, FR, RL, RR) in rpm, positive = forward.

    If a side would exceed max_wheel_rpm both sides are scaled down together, which keeps the turning radius.
    Non-finite inputs give zero.
    """
    if not (math.isfinite(linear_x) and math.isfinite(angular_z)):
        return (0.0, 0.0, 0.0, 0.0)
    half_track = geometry.track_width_m * geometry.turn_track_factor / 2.0
    rpm_per_mps = 60.0 / (math.pi * geometry.wheel_diameter_m)
    left = (linear_x - angular_z * half_track) * rpm_per_mps
    right = (linear_x + angular_z * half_track) * rpm_per_mps
    peak = max(abs(left), abs(right))
    if peak > geometry.max_wheel_rpm:
        scale = geometry.max_wheel_rpm / peak
        left, right = left * scale, right * scale
    return (left, right, left, right)
