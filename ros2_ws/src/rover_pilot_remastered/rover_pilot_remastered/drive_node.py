"""Rover-side drive node: /cmd_vel + the base station's e-stop heartbeat -> DRIVE_COMMAND to the drive ESP32 at 50 Hz.

Replaces the old rover_pilot drive_node. It owns the ESP32's serial port, never scans other ports, keeps sending
commands at a fixed rate (the firmware stops the motors 300 ms after the last one), and sends the e-stop flag whenever
the base station's e-stop is latched or its heartbeat is missing.
"""
from __future__ import annotations

import time

import rclpy
import serial
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from rover_pilot_remastered import protocol
from rover_pilot_remastered.drive_logic import DriveGate
from rover_pilot_remastered.kinematics import DriveGeometry, twist_to_wheel_rpm

LATEST_ONLY = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE)


def open_port(port: str, baud: int) -> serial.Serial:
    handle = serial.Serial()
    handle.port, handle.baudrate, handle.timeout, handle.write_timeout = port, baud, 0, 0.1
    handle.dtr = handle.rts = False  # avoid pulsing the ESP32 DevKit's reset circuit when opening
    try:
        handle.exclusive = True  # POSIX: refuse a second opener such as a serial monitor
    except (AttributeError, ValueError):
        pass
    handle.open()
    return handle


class DriveNode(Node):
    def __init__(self) -> None:
        super().__init__('drive_node')
        param = self.declare_parameter
        self.port = param('port', '/dev/rover-drive').value
        self.baud = param('baud', 115200).value
        rate_hz = param('rate_hz', 50.0).value
        self.gate = DriveGate(param('cmd_timeout_s', 0.3).value, param('estop_timeout_s', 0.5).value,
                              param('require_estop_heartbeat', True).value)
        self.geometry = DriveGeometry(param('wheel_diameter_m', 0.28).value, param('track_width_m', 0.84).value,
                                      param('max_wheel_rpm', 94.7).value, param('turn_track_factor', 1.0).value)
        param('wheel_width_m', 0.14)  # informational: recorded with the wheel profile
        self.swap_sides = param('swap_sides', False).value

        self.create_subscription(Twist, param('cmd_vel_topic', '/cmd_vel').value, self.on_cmd_vel, LATEST_ONLY)
        self.create_subscription(Bool, param('estop_topic', '/rover/estop').value, self.on_estop, LATEST_ONLY)
        self.diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        self.serial_port: serial.Serial | None = None
        self.next_open_attempt = 0.0
        self.port_error: str | None = None
        self.decoder = protocol.PacketDecoder()
        self.seq = 0
        self.status: protocol.DriveStatus | None = None
        self.status_time: float | None = None
        self.last_reason: str | None = None
        self.last_state: int | None = None
        self.warned_rpm_mismatch = False
        self.sent = 0

        self.create_timer(1.0 / rate_hz, self.tick)
        self.create_timer(1.0, self.publish_diagnostics)
        self.get_logger().info(
            f'drive node: port {self.port} @ {self.baud}, wheel {self.geometry.wheel_diameter_m} m, track '
            f'{self.geometry.track_width_m} m, max {self.geometry.max_linear_speed:.2f} m/s '
            f'({self.geometry.max_wheel_rpm} wheel rpm at full PWM)')

    def on_cmd_vel(self, msg: Twist) -> None:
        self.gate.on_cmd_vel(msg.linear.x, msg.angular.z, time.monotonic())

    def on_estop(self, msg: Bool) -> None:
        self.gate.on_estop(msg.data, time.monotonic())

    def tick(self) -> None:
        now = time.monotonic()
        decision = self.gate.decide(now)
        if decision.reason != self.last_reason:
            if decision.reason:
                self.get_logger().warn(f'stopped: {decision.reason}')
            else:
                self.get_logger().info('driving enabled')
            self.last_reason = decision.reason
        targets = twist_to_wheel_rpm(decision.linear_x, decision.angular_z, self.geometry)
        if self.swap_sides:
            targets = (targets[1], targets[0], targets[3], targets[2])
        self.send(protocol.drive_command_payload(decision.estop, targets))
        self.receive()

    def ensure_port(self) -> bool:
        if self.serial_port is not None:
            return True
        now = time.monotonic()
        if now < self.next_open_attempt:
            return False
        self.next_open_attempt = now + 1.0
        try:
            self.serial_port = open_port(self.port, self.baud)
        except (serial.SerialException, OSError) as error:
            if str(error) != self.port_error:
                self.get_logger().error(f'cannot open {self.port}: {error} (retrying every second)')
                self.port_error = str(error)
            return False
        self.port_error = None
        self.decoder = protocol.PacketDecoder()
        self.get_logger().info(f'serial port {self.port} open')
        return True

    def drop_port(self, error: Exception) -> None:
        self.get_logger().error(f'serial port {self.port} lost: {error}')
        try:
            if self.serial_port is not None:
                self.serial_port.close()
        except (serial.SerialException, OSError):
            pass
        self.serial_port = None
        self.status = None

    def send(self, payload: bytes) -> None:
        if not self.ensure_port():
            return
        try:
            self.serial_port.write(protocol.encode_packet(protocol.TYPE_DRIVE_COMMAND, self.seq, payload))
            self.seq = (self.seq + 1) & 0xFF
            self.sent += 1
        except (serial.SerialException, OSError) as error:
            self.drop_port(error)

    def receive(self) -> None:
        if self.serial_port is None:
            return
        try:
            chunk = self.serial_port.read(self.serial_port.in_waiting or 0)
        except (serial.SerialException, OSError, TypeError) as error:
            self.drop_port(error)
            return
        for packet in self.decoder.feed(chunk):
            status = protocol.parse_drive_status(packet.payload) if packet.type == protocol.TYPE_DRIVE_STATUS else None
            if status is None:
                continue
            if status.state != self.last_state:
                self.get_logger().info(f'ESP32 state: {status.state_name}')
                self.last_state = status.state
            if not self.warned_rpm_mismatch and abs(status.max_wheel_rpm - self.geometry.max_wheel_rpm) > 0.05:
                self.get_logger().warn(f'firmware MAX_WHEEL_RPM is {status.max_wheel_rpm} but max_wheel_rpm parameter is '
                                       f'{self.geometry.max_wheel_rpm}: speeds will be scaled wrongly')
                self.warned_rpm_mismatch = True
            self.status, self.status_time = status, time.monotonic()

    def publish_diagnostics(self) -> None:
        now = time.monotonic()
        status, decision = self.status, self.gate.decide(now)
        entry = DiagnosticStatus(name='rover drive', hardware_id=self.port)
        if self.serial_port is None:
            entry.level, entry.message = DiagnosticStatus.ERROR, f'serial port not open: {self.port_error or "opening"}'
        elif status is None or self.status_time is None or now - self.status_time > 0.5:
            entry.level, entry.message = DiagnosticStatus.ERROR, 'no status from the ESP32'
        elif status.state_name == 'DRIVING' and not decision.reason:
            entry.level, entry.message = DiagnosticStatus.OK, 'driving'
        else:
            entry.level, entry.message = DiagnosticStatus.WARN, decision.reason or status.state_name.lower()
        values = {'gate': decision.reason or 'driving', 'commands_sent': self.sent}
        if status is not None:
            values.update({'esp_state': status.state_name, 'command_age_ms': status.command_age_ms,
                           'pwm_fl_fr_rl_rr': ' '.join(str(p) for p in status.pwm), 'esp_rx_good': status.rx_good,
                           'esp_rx_bad': status.rx_bad, 'esp_uptime_s': round(status.uptime_ms / 1000, 1),
                           'esp_reset_reason': status.reset_reason, 'firmware': '.'.join(map(str, status.firmware_version))})
        entry.values = [KeyValue(key=key, value=str(value)) for key, value in values.items()]
        message = DiagnosticArray(status=[entry])
        message.header.stamp = self.get_clock().now().to_msg()
        self.diagnostics.publish(message)

    def stop_motors(self) -> None:
        """Best effort on shutdown: e-stop right away instead of waiting for the firmware's 300 ms timeout."""
        for _ in range(3):
            self.send(protocol.drive_command_payload(True, (0.0, 0.0, 0.0, 0.0)))
            time.sleep(0.01)
        if self.serial_port is not None:
            self.serial_port.close()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DriveNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_motors()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
