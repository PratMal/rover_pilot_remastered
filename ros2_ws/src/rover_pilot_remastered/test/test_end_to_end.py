"""Drive gate + kinematics + protocol over a real pseudo-terminal to the simulated ESP32 (no ROS needed)."""
import os
import time

import pytest
import serial

from rover_pilot_remastered import fake_esp, protocol
from rover_pilot_remastered.drive_logic import DriveGate
from rover_pilot_remastered.kinematics import DriveGeometry, twist_to_wheel_rpm

pytestmark = pytest.mark.skipif(not hasattr(os, 'openpty'), reason='needs POSIX pseudo-terminals')
ROVER = DriveGeometry(0.28, 0.84, 94.7)


def run(port, gate, seconds, decoder, seq=[0]):
    status, end = None, time.monotonic() + seconds
    while time.monotonic() < end:
        decision = gate.decide(time.monotonic())
        targets = twist_to_wheel_rpm(decision.linear_x, decision.angular_z, ROVER)
        port.write(protocol.encode_packet(protocol.TYPE_DRIVE_COMMAND, seq[0], protocol.drive_command_payload(decision.estop, targets)))
        seq[0] = (seq[0] + 1) & 0xFF
        for packet in decoder.feed(port.read(port.in_waiting or 0)):
            status = protocol.parse_drive_status(packet.payload) or status
        time.sleep(0.02)
    return status


def test_full_link_including_timeout_and_neutral_interlock():
    fake, path = fake_esp.start_in_background()
    port = serial.Serial(path, 115200, timeout=0)
    decoder = protocol.PacketDecoder()
    gate = DriveGate()
    try:
        now = time.monotonic()
        gate.on_estop(False, now)
        gate.on_cmd_vel(ROVER.max_linear_speed, 0.0, now)
        # Stick already pushed when the link comes up: the ESP32 must wait for neutral.
        status = run(port, gate, 0.25, decoder)
        assert status.state_name == 'WAIT_NEUTRAL' and status.pwm == (0, 0, 0, 0)
        gate.on_cmd_vel(0.0, 0.0, time.monotonic())
        gate.on_estop(False, time.monotonic())
        run(port, gate, 0.1, decoder)
        gate.on_cmd_vel(ROVER.max_linear_speed / 2, 0.0, time.monotonic())
        gate.on_estop(False, time.monotonic())
        status = run(port, gate, 0.25, decoder)
        assert status.state_name == 'DRIVING' and all(126 <= p <= 129 for p in status.pwm)
        # Base station disappears: heartbeat stale -> e-stop flag -> motors off.
        status = run(port, gate, 0.8, decoder)
        assert status.state_name == 'ESTOP' and status.pwm == (0, 0, 0, 0)
        # The drive node itself dies: no commands at all -> firmware timeout.
        gate.on_estop(False, time.monotonic())
        gate.on_cmd_vel(0.0, 0.0, time.monotonic())
        run(port, gate, 0.1, decoder)
        time.sleep(0.5)
        for packet in decoder.feed(port.read(port.in_waiting or 0)):
            status = protocol.parse_drive_status(packet.payload) or status
        assert status.state_name == 'NO_COMMAND'
        assert status.rx_bad == 0
    finally:
        fake.stop_event.set()
        port.close()
