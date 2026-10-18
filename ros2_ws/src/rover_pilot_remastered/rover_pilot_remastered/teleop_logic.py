"""Joystick -> /cmd_vel and the software e-stop latch, without ROS so it can be unit-tested.

E-stop rules:
  - starts latched: nothing moves until the operator releases it;
  - the e-stop button latches it at any time;
  - the release button clears it only while both sticks are centred (no lurch on release);
  - losing the joystick (no /joy for joy_timeout_s) latches it.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class TeleopConfig:
    # Defaults for a PS5 DualSense through joy's game_controller_node (SDL GameController layout).
    axis_linear: int = 1  # left stick vertical: up = +1
    axis_angular: int = 2  # right stick horizontal: left = +1 (joy inverts SDL so left/up are positive)
    invert_linear: bool = False
    invert_angular: bool = False
    deadzone: float = 0.08
    max_linear_speed: float = 1.389  # m/s at full stick (5 km/h)
    max_angular_speed: float = 3.307  # rad/s at full stick (spin in place at full PWM)
    slow_button: int = 9  # L1 held = slow mode; -1 disables
    slow_scale: float = 0.35
    estop_button: int = 1  # Circle
    release_button: int = 6  # Options
    joy_timeout_s: float = 0.5


def apply_deadzone(value: float, deadzone: float) -> float:
    if not math.isfinite(value) or abs(value) <= deadzone:
        return 0.0
    magnitude = min(1.0, (abs(value) - deadzone) / (1.0 - deadzone))
    return math.copysign(magnitude, value)


class TeleopLogic:
    def __init__(self, config: TeleopConfig) -> None:
        self.config = config
        self.estopped = True
        self.reason = 'startup: press release with both sticks centred'
        self._axes: list[float] = []
        self._buttons: list[int] = []
        self._last_joy: float | None = None
        self._release_held = False

    def _axis(self, index: int, invert: bool) -> float:
        if index < 0 or index >= len(self._axes):
            return 0.0
        value = apply_deadzone(float(self._axes[index]), self.config.deadzone)
        return -value if invert else value

    def _button(self, index: int) -> bool:
        return 0 <= index < len(self._buttons) and bool(self._buttons[index])

    def sticks_centred(self) -> bool:
        return self._axis(self.config.axis_linear, False) == 0.0 and self._axis(self.config.axis_angular, False) == 0.0

    def on_joy(self, axes: list[float], buttons: list[int], now: float) -> list[str]:
        """Feed one Joy message. Returns human-readable events for the log."""
        self._axes, self._buttons, self._last_joy = list(axes), list(buttons), now
        events = []
        if self._button(self.config.estop_button):
            if not self.estopped:
                events.append('E-STOP latched (button)')
            self.estopped, self.reason = True, 'e-stop button'
        release = self._button(self.config.release_button)
        if release and not self._release_held and self.estopped and not self._button(self.config.estop_button):
            if self.sticks_centred():
                self.estopped, self.reason = False, ''
                events.append('E-stop released: driving enabled')
            else:
                events.append('Release ignored: centre both sticks first')
        self._release_held = release
        return events

    def joystick_fresh(self, now: float) -> bool:
        return self._last_joy is not None and now - self._last_joy <= self.config.joy_timeout_s

    def tick(self, now: float) -> tuple[float, float, list[str]]:
        """Returns (linear m/s, angular rad/s, events). Call at the publish rate."""
        events = []
        if not self.joystick_fresh(now) and not self.estopped:
            events.append('E-STOP latched: joystick lost')
            self.estopped, self.reason = True, 'joystick lost'
        if self.estopped:
            return 0.0, 0.0, events
        scale = self.config.slow_scale if self._button(self.config.slow_button) else 1.0
        linear = self._axis(self.config.axis_linear, self.config.invert_linear) * self.config.max_linear_speed * scale
        angular = self._axis(self.config.axis_angular, self.config.invert_angular) * self.config.max_angular_speed * scale
        return linear, angular, events
