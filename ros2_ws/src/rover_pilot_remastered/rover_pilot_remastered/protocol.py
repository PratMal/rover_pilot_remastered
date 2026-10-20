"""Serial protocol between the drive node (onboard PC) and the drive ESP32.

Framing is identical to the rover's planned CAN-master link: each packet is
``version | type | seq | payload | crc16-le``, COBS-encoded and terminated by a single 0x00 byte.
The firmware implements the same protocol in firmware/lib/drive_core (C++).

Packets:
  0x82 DRIVE_COMMAND  PC -> ESP32, 50 Hz: flags (bit 0 = e-stop) + four wheel targets (FL, FR, RL, RR) in 0.1 rpm
  0x04 DRIVE_STATUS   ESP32 -> PC, 10 Hz: state, command age, applied PWM, link counters, uptime, reset reason, version
"""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass

PROTOCOL_VERSION = 1
TYPE_DRIVE_STATUS = 0x04
TYPE_DRIVE_COMMAND = 0x82
FLAG_ESTOP = 0x01
MAX_PACKET = 64  # decoded bytes: header + payload + CRC
WHEELS = ('FL', 'FR', 'RL', 'RR')

# Firmware states reported in DRIVE_STATUS (see firmware/lib/drive_core/src/drive_core.h).
STATE_NAMES = {0: 'BOOT', 1: 'NO_COMMAND', 2: 'ESTOP', 3: 'WAIT_NEUTRAL', 4: 'DRIVING'}

_COMMAND = struct.Struct('<B4h')
_STATUS = struct.Struct('<BBH4hHHIBH3B')


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF); crc16(b'123456789') == 0x29B1."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0])
    code_index, code = 0, 1
    for byte in data:
        if byte == 0:
            out[code_index] = code
            code_index, code = len(out), 1
            out.append(0)
        else:
            out.append(byte)
            code += 1
            if code == 0xFF:
                out[code_index] = code
                code_index, code = len(out), 1
                out.append(0)
    out[code_index] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes | None:
    out = bytearray()
    index = 0
    while index < len(data):
        code = data[index]
        index += 1
        if code == 0 or index + code - 1 > len(data) or 0 in data[index:index + code - 1]:
            return None
        out += data[index:index + code - 1]
        index += code - 1
        if code != 0xFF and index < len(data):
            out.append(0)
    return bytes(out)


def encode_packet(packet_type: int, seq: int, payload: bytes) -> bytes:
    """Complete wire bytes for one packet, including the trailing 0x00 delimiter."""
    body = bytes([PROTOCOL_VERSION, packet_type & 0xFF, seq & 0xFF]) + payload
    if len(body) + 2 > MAX_PACKET:
        raise ValueError('packet too long')
    return cobs_encode(body + struct.pack('<H', crc16(body))) + b'\x00'


@dataclass(frozen=True)
class Packet:
    type: int
    seq: int
    payload: bytes


class PacketDecoder:
    """Feed arbitrary byte chunks; get back complete packets that pass COBS, CRC and version checks."""

    def __init__(self) -> None:
        self.good = 0
        self.bad = 0
        self._buffer = bytearray()
        self._overflow = False
        self._first = True

    def feed(self, chunk: bytes) -> list[Packet]:
        packets = []
        for byte in chunk:
            if byte:
                if len(self._buffer) <= MAX_PACKET:
                    self._buffer.append(byte)
                else:
                    self._overflow = True
                continue
            block, overflow, first = bytes(self._buffer), self._overflow, self._first
            self._buffer, self._overflow, self._first = bytearray(), False, False
            if not block:
                continue
            packet = None if overflow else self._decode(block)
            if packet is not None:
                self.good += 1
                packets.append(packet)
            elif not first:  # the first block may be the tail of a packet we joined mid-stream
                self.bad += 1
        return packets

    @staticmethod
    def _decode(block: bytes) -> Packet | None:
        decoded = cobs_decode(block)
        if decoded is None or len(decoded) < 5:
            return None
        body, crc = decoded[:-2], struct.unpack('<H', decoded[-2:])[0]
        if crc16(body) != crc or body[0] != PROTOCOL_VERSION:
            return None
        return Packet(type=body[1], seq=body[2], payload=bytes(body[3:]))


def drive_command_payload(estop: bool, targets_rpm: list[float] | tuple[float, ...]) -> bytes:
    """Payload of DRIVE_COMMAND: wheel targets FL, FR, RL, RR in rpm (positive = forward), 0.1 rpm resolution.
    A non-finite target is sent as 0: the drive node's 50 Hz loop must never fail on a bad value."""
    if len(targets_rpm) != 4:
        raise ValueError('need four wheel targets (FL, FR, RL, RR)')
    raw = [max(-32767, min(32767, round(rpm * 10))) if math.isfinite(rpm) else 0 for rpm in targets_rpm]
    return _COMMAND.pack(FLAG_ESTOP if estop else 0, *raw)


def parse_drive_command(payload: bytes) -> tuple[bool, tuple[float, ...]] | None:
    """(estop, targets rpm), or None if the length is wrong or a reserved flag bit is set (as the firmware does)."""
    if len(payload) != _COMMAND.size:
        return None
    flags, *raw = _COMMAND.unpack(payload)
    if flags & ~FLAG_ESTOP:
        return None
    return bool(flags & FLAG_ESTOP), tuple(value / 10 for value in raw)


@dataclass(frozen=True)
class DriveStatus:
    state: int
    command_age_ms: int
    pwm: tuple[int, int, int, int]  # applied duty -255..255, FL FR RL RR (positive = forward)
    rx_good: int
    rx_bad: int
    uptime_ms: int
    reset_reason: int  # esp_reset_reason_t of the last boot
    max_wheel_rpm: float  # firmware's open-loop calibration: PWM 255 == this wheel rpm
    firmware_version: tuple[int, int, int]

    @property
    def state_name(self) -> str:
        return STATE_NAMES.get(self.state, f'UNKNOWN({self.state})')


def drive_status_payload(status: DriveStatus) -> bytes:
    return _STATUS.pack(status.state, 0, min(status.command_age_ms, 0xFFFF), *status.pwm, min(status.rx_good, 0xFFFF),
                        min(status.rx_bad, 0xFFFF), status.uptime_ms & 0xFFFFFFFF, status.reset_reason,
                        round(status.max_wheel_rpm * 10), *status.firmware_version)


def parse_drive_status(payload: bytes) -> DriveStatus | None:
    if len(payload) != _STATUS.size:
        return None
    state, _flags, age, fl, fr, rl, rr, good, bad, uptime, reset, max_rpm_x10, major, minor, patch = _STATUS.unpack(payload)
    return DriveStatus(state, age, (fl, fr, rl, rr), good, bad, uptime, reset, max_rpm_x10 / 10, (major, minor, patch))
