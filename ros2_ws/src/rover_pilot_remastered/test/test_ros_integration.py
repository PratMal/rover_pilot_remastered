"""Joystick -> base_teleop -> /cmd_vel + /rover/estop -> drive_node -> serial -> simulated ESP32, through real ROS 2.

drive_node and base_teleop run as separate processes, as on the rover and the base station; this test publishes
/joy and watches the simulated ESP32. Needs a sourced ROS 2 environment (skipped otherwise):
    source /opt/ros/humble/setup.bash && python3 -m pytest test/test_ros_integration.py
"""
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')
from sensor_msgs.msg import Joy  # noqa: E402

from rover_pilot_remastered import fake_esp  # noqa: E402

pytestmark = pytest.mark.skipif(not hasattr(os, 'openpty'), reason='needs POSIX pseudo-terminals')
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CIRCLE, OPTIONS = 1, 6
STOPPED = (0, 0, 0, 0)


def start(module: str, *ros_args: str) -> subprocess.Popen:
    env = dict(os.environ, PYTHONPATH=os.pathsep.join([str(PACKAGE_ROOT), os.environ.get('PYTHONPATH', '')]))
    return subprocess.Popen([sys.executable, '-m', f'rover_pilot_remastered.{module}', '--ros-args', *ros_args],
                            env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def test_joystick_to_wheels_through_ros():
    fake, path = fake_esp.start_in_background()
    processes = [start('drive_node', '-p', f'port:={path}'), start('base_teleop')]
    rclpy.init()
    node = rclpy.create_node('fake_joystick')
    publisher = node.create_publisher(Joy, '/joy', 1)

    def esp():
        with fake.lock:
            return fake.controller.state, fake.controller.pwm()

    def hold(until=None, timeout=1.0, left_y=0.0, right_x=0.0, pressed=()):
        """Publish one joystick state at 50 Hz until `until(esp())` holds (returns the elapsed time) or the timeout."""
        started = time.monotonic()
        while time.monotonic() - started < timeout:
            message = Joy()
            message.axes = [0.0, left_y, right_x, 0.0, 1.0, 1.0]
            message.buttons = [1 if index in pressed else 0 for index in range(15)]
            publisher.publish(message)
            if until is not None and until(esp()):
                return time.monotonic() - started
            time.sleep(0.02)
        return None

    try:
        # Discovery; the teleop starts latched, so the ESP32 is told to e-stop.
        assert hold(lambda s: s == (fake_esp.ESTOP, STOPPED), timeout=15.0) is not None, esp()

        hold(timeout=0.2, pressed=(OPTIONS,))  # release with the sticks centred
        assert hold(lambda s: s == (fake_esp.DRIVING, STOPPED), timeout=2.0) is not None, esp()

        assert hold(lambda s: s == (fake_esp.DRIVING, (255, 255, 255, 255)), timeout=2.0, left_y=1.0) is not None, esp()
        # Stick right = turn right: left wheels forward, right wheels backward.
        assert hold(lambda s: s == (fake_esp.DRIVING, (255, -255, 255, -255)), timeout=2.0, right_x=-1.0) is not None, esp()

        latency = hold(lambda s: s == (fake_esp.ESTOP, STOPPED), timeout=2.0, left_y=1.0, pressed=(CIRCLE,))
        assert latency is not None and latency < 0.6, (latency, esp())

        hold(timeout=0.6, left_y=1.0, pressed=(OPTIONS,))  # release refused while the stick is pushed
        assert esp() == (fake_esp.ESTOP, STOPPED)

        hold(timeout=0.3)
        hold(timeout=0.2, pressed=(OPTIONS,))
        assert hold(lambda s: s[0] == fake_esp.DRIVING and all(100 < p < 150 for p in s[1]), timeout=2.0, left_y=0.5) is not None, esp()

        # The base station goes silent: joystick lost / heartbeat stale -> e-stop.
        started = time.monotonic()
        while esp() != (fake_esp.ESTOP, STOPPED) and time.monotonic() - started < 3.0:
            time.sleep(0.02)
        assert esp() == (fake_esp.ESTOP, STOPPED)
        assert time.monotonic() - started < 1.0
    finally:
        for process in processes:
            process.send_signal(signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
        node.destroy_node()
        rclpy.shutdown()
        fake.stop_event.set()
