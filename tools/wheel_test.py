#!/usr/bin/env python3
"""Wheel identification test for the drive ESP32, without ROS. WHEELS OFF THE GROUND.

Spins each wheel forward on its own, then all forward, then a right turn, printing what the ESP32 reports. Use it after
flashing to check that FL/FR/RL/RR are the wheels you think they are and that "forward" really is forward.
Needs only pyserial (pip install pyserial). Stop the drive node first: only one program may use the port.

    python3 tools/wheel_test.py /dev/rover-drive            # default 30 % of full speed
    python3 tools/wheel_test.py /dev/ttyUSB0 --percent 50
"""
import argparse
import sys
import time
from pathlib import Path

import serial

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws' / 'src' / 'rover_pilot_remastered'))
from rover_pilot_remastered import protocol  # noqa: E402

MAX_WHEEL_RPM = 94.7


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('port')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--percent', type=float, default=30.0, help='speed as a percentage of full PWM')
    parser.add_argument('--seconds', type=float, default=2.0, help='duration of each step')
    args = parser.parse_args()
    rpm = MAX_WHEEL_RPM * max(0.0, min(100.0, args.percent)) / 100.0

    port = serial.Serial()
    port.port, port.baudrate, port.timeout = args.port, args.baud, 0
    port.dtr = port.rts = False
    port.open()
    decoder, seq = protocol.PacketDecoder(), 0
    status = None

    def run(label: str, estop: bool, targets: tuple, seconds: float) -> None:
        nonlocal seq, status
        print(f'\n>>> {label}')
        end, next_print = time.monotonic() + seconds, 0.0
        while time.monotonic() < end:
            port.write(protocol.encode_packet(protocol.TYPE_DRIVE_COMMAND, seq, protocol.drive_command_payload(estop, targets)))
            seq = (seq + 1) & 0xFF
            for packet in decoder.feed(port.read(port.in_waiting or 0)):
                status = protocol.parse_drive_status(packet.payload) or status
            if status and time.monotonic() >= next_print:
                next_print = time.monotonic() + 0.5
                fl, fr, rl, rr = status.pwm
                print(f'    ESP32 {status.state_name:<12} PWM  FL {fl:4}  FR {fr:4}  RL {rl:4}  RR {rr:4}   '
                      f'(fw {".".join(map(str, status.firmware_version))}, bad packets {status.rx_bad})')
            time.sleep(0.02)

    zero = (0.0, 0.0, 0.0, 0.0)
    steps = [('neutral (required before any motion)', False, zero, 1.0)]
    for index, wheel in enumerate(protocol.WHEELS):
        targets = [0.0] * 4
        targets[index] = rpm
        steps += [(f'{wheel} forward only', False, tuple(targets), args.seconds), ('stop', False, zero, 0.7)]
    steps += [('all forward', False, (rpm,) * 4, args.seconds), ('stop', False, zero, 0.7),
              ('turn right in place (left wheels forward, right wheels backward)', False, (rpm, -rpm, rpm, -rpm), args.seconds),
              ('e-stop', True, zero, 1.0)]
    try:
        for step in steps:
            run(*step)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            port.write(protocol.encode_packet(protocol.TYPE_DRIVE_COMMAND, seq, protocol.drive_command_payload(True, zero)))
        port.close()
    if status is None:
        print('\nNo DRIVE_STATUS received: wrong port or baud, or the new firmware is not flashed.')
        return 1
    print('\nDone. If a wheel was wrong, fix firmware/include/config.h (pins / forward_level); if only turning was '
          'mirrored, set swap_sides or swap the left/right pin entries.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
