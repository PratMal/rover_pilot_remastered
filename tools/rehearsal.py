#!/usr/bin/env python3
"""Dress rehearsal of the bench test (README, "First test on the bench" and "Daily workflow") without hardware.

Runs the real launch files and parameter files (`ros2 launch ... rover.launch.py` / `base_station.launch.py`) against
the simulated ESP32, presses the PS5 buttons in the README's order, and times every stop. Stand-ins: the ESP32 is
fake_esp on a pseudo-terminal inside this process; the PS5 is this script publishing /joy the way joy's
game_controller_node does (20 Hz autorepeat); "unplugging" either of them stops it. Nothing touches real hardware.

Needs ROS 2 Humble and this workspace built and sourced, with the RMW and ROS_DOMAIN_ID you use on the rover:
    source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
    python3 tools/rehearsal.py                       # every step; PASS/FAIL table, exit code 1 on any failure
    python3 tools/rehearsal.py --wheels wheel_other
"""
from __future__ import annotations

import argparse
import os
import re
import signal
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws' / 'src' / 'rover_pilot_remastered'))

import rclpy  # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from sensor_msgs.msg import Joy  # noqa: E402

from rover_pilot_remastered import fake_esp  # noqa: E402

LINK = '/tmp/rover-drive-rehearsal'
CIRCLE, OPTIONS, L1 = 1, 6, 9  # config/drive.yaml defaults (PS5 through game_controller_node)
STOPPED = (0, 0, 0, 0)
NAMES = fake_esp.protocol.STATE_NAMES


class SimulatedEsp:
    """fake_esp on a fresh pseudo-terminal, reachable at LINK (the stand-in for /dev/rover-drive)."""

    def __init__(self) -> None:
        self.fd, path = fake_esp.open_pty()
        self.fake = fake_esp.FakeEsp(self.fd)
        self.thread = threading.Thread(target=self.fake.run, name='fake-esp', daemon=True)
        self.thread.start()
        link = Path(LINK)
        if link.is_symlink() or link.exists():
            link.unlink()
        link.symlink_to(path)

    def read(self) -> tuple[int, tuple[int, ...]]:
        with self.fake.lock:
            return self.fake.controller.state, self.fake.controller.pwm()

    def reset(self) -> None:
        """The EN button: the ESP32 reboots, USB stays connected."""
        with self.fake.lock:
            self.fake.controller = fake_esp.FakeController(self.fake.controller.max_wheel_rpm)

    def unplug(self) -> None:
        self.fake.stop_event.set()
        self.thread.join(2)
        os.close(self.fd)  # the drive node's port now fails, as when the USB cable is pulled
        Path(LINK).unlink(missing_ok=True)


class Ps5:
    """Publishes /joy at 20 Hz like game_controller_node with autorepeat, while plugged in."""

    def __init__(self, node) -> None:
        self.publisher = node.create_publisher(Joy, '/joy', 10)
        self.left_y = self.right_x = 0.0
        self.pressed: set[int] = set()
        self.plugged = True
        node.create_timer(0.05, self.publish)

    def publish(self) -> None:
        if not self.plugged:
            return
        message = Joy()
        message.axes = [0.0, self.left_y, self.right_x, 0.0, 1.0, 1.0]
        message.buttons = [1 if index in self.pressed else 0 for index in range(15)]
        self.publisher.publish(message)

    def set(self, left_y: float = 0.0, right_x: float = 0.0, pressed: tuple[int, ...] = ()) -> None:
        self.left_y, self.right_x, self.pressed = left_y, right_x, set(pressed)

    def press(self, button: int, hold_s: float = 0.25, **sticks: float) -> None:
        self.set(pressed=(button,), **sticks)
        time.sleep(hold_s)
        self.set(**sticks)


class Launch:
    """One `ros2 launch` in its own process group, so Ctrl-C reaches every node like in a terminal."""

    def __init__(self, name: str, logs: Path, *arguments: str) -> None:
        self.name, self.log = name, logs / f'{name}.log'
        environment = dict(os.environ, PYTHONUNBUFFERED='1', RCUTILS_LOGGING_BUFFERED_STREAM='0')  # logs in order, at once
        self.process = subprocess.Popen(['ros2', 'launch', 'rover_pilot_remastered', *arguments], stdout=self.log.open('a'),
                                        stderr=subprocess.STDOUT, start_new_session=True, env=environment)

    def interrupt(self) -> None:
        """Ctrl-C in the launch's terminal: SIGINT to the whole process group."""
        os.killpg(self.process.pid, signal.SIGINT)

    def wait_exit(self) -> int | None:
        try:
            return self.process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            self.kill()
            return None

    def ctrl_c(self) -> int | None:
        self.interrupt()
        return self.wait_exit()

    def kill(self) -> None:
        """The laptop dies: no shutdown code runs."""
        try:
            os.killpg(self.process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        self.process.wait(timeout=10)

    def text(self) -> str:
        return self.log.read_text(errors='replace')


class Rehearsal:
    def __init__(self, wheels: str, logs: Path) -> None:
        self.wheels, self.logs = wheels, logs
        self.results: list[tuple[str, bool, str]] = []
        rclpy.init()
        self.node = rclpy.create_node('rehearsal_ps5')
        self.ps5 = Ps5(self.node)
        self.diagnostics: dict[str, str] = {}
        self.node.create_subscription(DiagnosticArray, '/diagnostics', self.on_diagnostics, 10)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.spinner = threading.Thread(target=self.executor.spin, daemon=True)
        self.spinner.start()
        self.esp: SimulatedEsp | None = None
        self.rover: Launch | None = None
        self.base: Launch | None = None

    def on_diagnostics(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name == 'rover drive':
                self.diagnostics = {item.key: item.value for item in status.values} | {'_message': status.message}

    # ------------------------------------------------------------------ helpers
    def state(self) -> tuple[int, tuple[int, ...]] | None:
        return self.esp.read() if self.esp else None

    def wait(self, condition, timeout: float) -> float | None:
        started = time.monotonic()
        while time.monotonic() - started < timeout:
            current = self.state()
            if current is not None and condition(current):
                return time.monotonic() - started
            time.sleep(0.005)
        return None

    def holds(self, condition, seconds: float) -> bool:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            current = self.state()
            if current is None or not condition(current):
                return False
            time.sleep(0.01)
        return True

    def describe(self) -> str:
        current = self.state()
        return 'no ESP32' if current is None else f'{NAMES[current[0]]} PWM {current[1]}'

    def check(self, name: str, ok: bool, detail: str = '') -> bool:
        self.results.append((name, bool(ok), detail or self.describe()))
        print(f"  {'PASS' if ok else 'FAIL'}  {name}: {detail or self.describe()}", flush=True)
        return bool(ok)

    def start_rover(self) -> None:
        self.rover = Launch('rover', self.logs, 'rover.launch.py', f'port:={LINK}', f'wheels:={self.wheels}')

    def start_base(self) -> None:
        self.base = Launch('base_station', self.logs, 'base_station.launch.py', f'wheels:={self.wheels}')

    def release(self) -> bool:
        """Centre the sticks and press Options, as the operator does; True once the ESP32 drives."""
        self.ps5.set()
        time.sleep(0.15)
        self.ps5.press(OPTIONS)
        return self.wait(lambda s: s == (fake_esp.DRIVING, STOPPED), 2.0) is not None

    def drive_forward(self, amount: float = 0.5) -> bool:
        self.ps5.set(left_y=amount)
        return self.wait(lambda s: s[0] == fake_esp.DRIVING and all(p > 0 for p in s[1]), 2.0) is not None

    def installed_matches_source(self) -> list[str]:
        """Files whose installed copy differs from the source: `ros2 launch` runs the installed package, so after an
        edit (code or config) it runs the old one until `colcon build` is run again."""
        from ament_index_python.packages import get_package_prefix, get_package_share_directory  # noqa: PLC0415

        source = ROOT / 'ros2_ws' / 'src' / 'rover_pilot_remastered'
        share = Path(get_package_share_directory('rover_pilot_remastered'))
        modules = next(Path(get_package_prefix('rover_pilot_remastered')).glob('lib/python3*/site-packages/rover_pilot_remastered'))
        pairs = [(path, modules / path.name) for path in (source / 'rover_pilot_remastered').glob('*.py')]
        pairs += [(path, share / path.relative_to(source)) for pattern in ('config/*.yaml', 'config/wheels/*.yaml', 'launch/*.py')
                  for path in source.glob(pattern)]
        return [str(src.relative_to(source)) for src, installed in pairs if not installed.exists() or installed.read_bytes() != src.read_bytes()]

    # ------------------------------------------------------------------ the run
    def run(self) -> bool:
        print(f'logs in {self.logs}', flush=True)
        stale = self.installed_matches_source()
        if not self.check('installed package matches the source', not stale, 'up to date' if not stale else
                          f"rebuild first (cd ros2_ws && colcon build --symlink-install): {', '.join(stale)}"):
            return False
        self.esp = SimulatedEsp()

        print('\n1. Wheel test (tools/wheel_test.py, drive node not running)', flush=True)
        result = subprocess.run([sys.executable, str(ROOT / 'tools' / 'wheel_test.py'), LINK, '--seconds', '0.6'],
                                capture_output=True, text=True, timeout=60)
        (self.logs / 'wheel_test.log').write_text(result.stdout + result.stderr)
        lines = result.stdout.splitlines()

        def step_pwm(label: str) -> tuple[int, ...] | None:
            """The first PWM report printed for the wheel test step `label`."""
            index = next((i for i, line in enumerate(lines) if line.startswith(f'>>> {label}')), None)
            line = next((line for line in lines[index + 1:] if 'PWM' in line), '') if index is not None else ''
            match = re.search(r'FL\s+(-?\d+)\s+FR\s+(-?\d+)\s+RL\s+(-?\d+)\s+RR\s+(-?\d+)', line)
            return tuple(int(value) for value in match.groups()) if match else None

        # 30 % of full PWM: 28.4 rpm on the wire, PWM 76 (rounded like the firmware).
        speed = round(round(94.7 * 0.3 * 10) / 10 / 94.7 * 255)
        self.check('wheel test exits cleanly', result.returncode == 0, f'exit {result.returncode}')
        self.check('wheel test reports fw 2.0.2', 'fw 2.0.2' in result.stdout, 'fw 2.0.2' if 'fw 2.0.2' in result.stdout else 'version missing')
        for index, wheel in enumerate(('FL', 'FR', 'RL', 'RR')):
            expected = tuple(speed if i == index else 0 for i in range(4))
            self.check(f'wheel test: {wheel} forward only', step_pwm(f'{wheel} forward only') == expected, str(step_pwm(f'{wheel} forward only')))
        self.check('wheel test: all forward', step_pwm('all forward') == (speed,) * 4, str(step_pwm('all forward')))
        self.check('wheel test: turn right in place', step_pwm('turn right in place') == (speed, -speed, speed, -speed),
                   str(step_pwm('turn right in place')))
        self.check('wheel test leaves the ESP32 in e-stop', self.wait(lambda s: s == (fake_esp.ESTOP, STOPPED), 1.0) is not None)
        self.esp.unplug()
        self.esp = SimulatedEsp()  # a freshly booted ESP32, so only the drive node can move it out of BOOT

        print('\n2. Daily workflow: launch the rover and the base station', flush=True)
        self.start_rover()
        self.start_base()
        # The teleop starts latched, so the first thing the ESP32 hears from the drive node is an e-stop.
        self.check('ESP32 boots with its motors off', self.state() == (fake_esp.BOOT, STOPPED))
        self.check('launches come up; the e-stop starts latched', self.wait(lambda s: s[0] == fake_esp.ESTOP, 30.0) is not None
                   and self.holds(lambda s: s[0] == fake_esp.ESTOP, 1.0))
        self.check('Options with the sticks centred: driving enabled', self.release())

        print('\n3. Joystick', flush=True)
        self.ps5.set(left_y=1.0)
        self.check('left stick up: every wheel forward at full PWM', self.wait(lambda s: s == (fake_esp.DRIVING, (255, 255, 255, 255)), 2.0) is not None)
        self.ps5.set(right_x=-1.0)
        self.check('right stick right: turns right (left wheels forward, right backward)',
                   self.wait(lambda s: s == (fake_esp.DRIVING, (255, -255, 255, -255)), 2.0) is not None)
        self.ps5.set(left_y=0.5, right_x=0.3)
        self.check('half stick forward, a little left: arc to the left',
                   self.wait(lambda s: s[0] == fake_esp.DRIVING and s[1][0] == s[1][2] and s[1][1] == s[1][3] and 0 < s[1][0] < s[1][1], 2.0) is not None)
        self.ps5.set(left_y=1.0, pressed=(L1,))
        self.check('L1 held: slow mode (35 %)', self.wait(lambda s: s[0] == fake_esp.DRIVING and all(85 <= p <= 93 for p in s[1]), 2.0) is not None)
        self.ps5.set()
        self.check('sticks released: wheels stop, still enabled', self.wait(lambda s: s == (fake_esp.DRIVING, STOPPED), 1.0) is not None)

        print('\n4. Stops', flush=True)
        self.drive_forward(1.0)
        self.ps5.set(left_y=1.0, pressed=(CIRCLE,))
        latency = self.wait(lambda s: s == (fake_esp.ESTOP, STOPPED), 2.0)
        self.check('Circle: e-stop', latency is not None and latency < 0.15, f'{latency:.3f} s' if latency is not None else self.describe())
        self.ps5.set(left_y=1.0)
        time.sleep(0.2)
        self.ps5.press(OPTIONS, left_y=1.0)
        self.check('Options with a stick pushed: refused', self.holds(lambda s: s == (fake_esp.ESTOP, STOPPED), 1.0))
        self.check('centred + Options: driving again', self.release())

        self.drive_forward()
        self.ps5.plugged = False
        latency = self.wait(lambda s: s == (fake_esp.ESTOP, STOPPED), 3.0)
        self.check('PS5 unplugged: e-stop', latency is not None and latency < 0.7, f'{latency:.3f} s' if latency is not None else self.describe())
        self.ps5.plugged = True
        self.ps5.set(left_y=0.5)
        self.check('PS5 back, stick pushed: still stopped', self.holds(lambda s: s == (fake_esp.ESTOP, STOPPED), 1.0))
        self.check('centred + Options after replugging: driving again', self.release())

        self.drive_forward()
        self.base.interrupt()
        latency = self.wait(lambda s: s == (fake_esp.ESTOP, STOPPED), 2.0)
        self.check('Ctrl-C on the base station launch: e-stop before the heartbeat timeout', latency is not None and latency < 0.4,
                   f'{latency:.3f} s' if latency is not None else self.describe())
        code = self.base.wait_exit()
        self.check('base station shutdown without a traceback', 'Traceback' not in self.base.text() and code == 0, f'launch exit {code}')
        self.start_base()
        time.sleep(3.0)
        self.check('base station restarted: still stopped until released', self.state() == (fake_esp.ESTOP, STOPPED))
        self.check('centred + Options: driving again', self.release())

        self.drive_forward()
        self.base.kill()  # the base station laptop dies, or the radio link drops
        latency = self.wait(lambda s: s == (fake_esp.ESTOP, STOPPED), 3.0)
        self.check('base station lost (no shutdown): e-stop from the heartbeat timeout', latency is not None and latency < 0.8,
                   f'{latency:.3f} s' if latency is not None else self.describe())
        self.start_base()
        time.sleep(3.0)
        self.check('base station back: driving again after Options', self.release())

        self.drive_forward()
        self.rover.interrupt()
        latency = self.wait(lambda s: s[1] == STOPPED, 2.0)
        self.check('Ctrl-C on the rover launch: wheels stop before the firmware timeout', latency is not None and latency < 0.3,
                   f'{latency:.3f} s, {self.describe()}' if latency is not None else self.describe())
        code = self.rover.wait_exit()
        self.check('the drive node e-stopped the ESP32 on its way out', self.state() == (fake_esp.ESTOP, STOPPED), f'{self.describe()}, launch exit {code}')
        self.check('rover shutdown without a traceback', 'Traceback' not in self.rover.text() and code == 0, f'launch exit {code}')
        self.ps5.set(left_y=0.5)
        self.start_rover()
        time.sleep(4.0)
        self.check('rover relaunched with a stick pushed: the ESP32 waits for neutral',
                   self.state() is not None and self.state()[1] == STOPPED and self.state()[0] in (fake_esp.ESTOP, fake_esp.WAIT_NEUTRAL))
        self.ps5.set()
        self.check('sticks centred: driving again', self.wait(lambda s: s == (fake_esp.DRIVING, STOPPED), 2.0) is not None)

        self.drive_forward()
        self.esp.unplug()  # the ESP32 loses USB power, so its motors stop with it
        time.sleep(1.5)
        self.check('ESP32 USB unplugged: the drive node notices and keeps running',
                   'lost' in self.rover.text() and self.rover.process.poll() is None, 'port lost logged' if 'lost' in self.rover.text() else 'not logged')
        self.esp = SimulatedEsp()  # plugged back in: the ESP32 boots
        self.check('USB back with the stick still pushed: the ESP32 waits for neutral',
                   self.wait(lambda s: s[0] == fake_esp.WAIT_NEUTRAL, 4.0) is not None and self.holds(lambda s: s[1] == STOPPED, 0.5))
        self.ps5.set()
        self.check('sticks centred: driving again', self.wait(lambda s: s == (fake_esp.DRIVING, STOPPED), 2.0) is not None)

        self.drive_forward()
        self.esp.reset()  # EN button with the stick pushed
        self.check('ESP32 reset (EN) with the stick pushed: it does not move until centred',
                   self.wait(lambda s: s[0] == fake_esp.WAIT_NEUTRAL, 1.0) is not None and self.holds(lambda s: s[1] == STOPPED, 1.0))
        self.ps5.set()
        self.check('sticks centred: driving again', self.wait(lambda s: s == (fake_esp.DRIVING, STOPPED), 2.0) is not None)

        print('\n5. Diagnostics', flush=True)
        self.drive_forward()
        time.sleep(1.5)
        diagnostics = dict(self.diagnostics)
        self.check('/diagnostics: firmware 2.0.2', diagnostics.get('firmware') == '2.0.2', diagnostics.get('firmware', 'none'))
        self.check('/diagnostics: esp_rx_bad stays 0', diagnostics.get('esp_rx_bad') == '0', diagnostics.get('esp_rx_bad', 'none'))
        self.check('/diagnostics: driving', diagnostics.get('_message') == 'driving', diagnostics.get('_message', 'none'))

        print('\n6. End of the session: Circle, then Ctrl-C on both machines', flush=True)
        self.ps5.press(CIRCLE)
        self.check('Circle: e-stop', self.wait(lambda s: s == (fake_esp.ESTOP, STOPPED), 1.0) is not None)
        self.base.ctrl_c()
        self.rover.ctrl_c()
        logs = self.base.text() + self.rover.text()
        self.check('no tracebacks in any launch log', 'Traceback' not in logs)
        return all(ok for _, ok, _ in self.results)

    def close(self) -> None:
        for launch in (self.base, self.rover):
            if launch is not None and launch.process.poll() is None:
                launch.ctrl_c()
        if self.esp is not None:
            self.esp.unplug()
        self.executor.shutdown()
        self.spinner.join(5)  # destroying the node under a spinning executor aborts the process
        self.node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--wheels', default='wheel_28cm', help='wheel profile for both launches (config/wheels)')
    args = parser.parse_args()
    logs = Path(tempfile.mkdtemp(prefix='rover-rehearsal-'))
    rehearsal = Rehearsal(args.wheels, logs)
    try:
        passed = rehearsal.run()
    except KeyboardInterrupt:
        passed = False
    finally:
        rehearsal.close()
    failed = [name for name, ok, _ in rehearsal.results if not ok]
    print(f"\n{len(rehearsal.results) - len(failed)}/{len(rehearsal.results)} checks passed" + (f"; failed: {', '.join(failed)}" if failed else ''))
    print(f'launch logs: {logs}')
    return 0 if passed and not failed else 1


if __name__ == '__main__':
    sys.exit(main())
