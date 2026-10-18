"""What the drive node sends each cycle, without ROS so it can be unit-tested.

The rover drives only while the base station's e-stop heartbeat is fresh and says "released" and /cmd_vel is fresh.
Anything else sends the e-stop flag (latched in the firmware until a neutral command) or zero targets.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Decision:
    estop: bool
    linear_x: float
    angular_z: float
    reason: str  # '' while driving normally


class DriveGate:
    def __init__(self, cmd_timeout_s: float = 0.3, estop_timeout_s: float = 0.5, require_estop_heartbeat: bool = True) -> None:
        self.cmd_timeout_s = cmd_timeout_s
        self.estop_timeout_s = estop_timeout_s
        self.require_estop_heartbeat = require_estop_heartbeat
        self._cmd: tuple[float, float] = (0.0, 0.0)
        self._cmd_time: float | None = None
        self._estop: bool = True
        self._estop_time: float | None = None

    def on_cmd_vel(self, linear_x: float, angular_z: float, now: float) -> None:
        self._cmd, self._cmd_time = (linear_x, angular_z), now

    def on_estop(self, stopped: bool, now: float) -> None:
        self._estop, self._estop_time = stopped, now

    def decide(self, now: float) -> Decision:
        if self.require_estop_heartbeat:
            if self._estop_time is None:
                return Decision(True, 0.0, 0.0, 'waiting for the base station e-stop heartbeat')
            if now - self._estop_time > self.estop_timeout_s:
                return Decision(True, 0.0, 0.0, 'base station e-stop heartbeat lost')
        if self._estop_time is not None and self._estop:
            return Decision(True, 0.0, 0.0, 'e-stop')
        if self._cmd_time is None or now - self._cmd_time > self.cmd_timeout_s:
            return Decision(False, 0.0, 0.0, 'no fresh /cmd_vel')
        return Decision(False, self._cmd[0], self._cmd[1], '')
