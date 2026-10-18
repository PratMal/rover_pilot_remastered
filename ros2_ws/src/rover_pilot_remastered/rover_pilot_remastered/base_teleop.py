"""Base-station teleop: PS5 controller (/joy) -> /cmd_vel (20 Hz) and the e-stop heartbeat /rover/estop (10 Hz).

/cmd_vel follows REP 103 (m/s, rad/s, +yaw = turn left), so Nav2 can later publish the same topic.
Full stick defaults to the rover's top speed, derived from the same wheel profile the rover uses.
"""
from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

from rover_pilot_remastered.kinematics import DriveGeometry
from rover_pilot_remastered.teleop_logic import TeleopConfig, TeleopLogic

LATEST_ONLY = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE)


class BaseTeleop(Node):
    def __init__(self) -> None:
        super().__init__('base_teleop')
        param = self.declare_parameter
        defaults = TeleopConfig()
        geometry = DriveGeometry(param('wheel_diameter_m', 0.28).value, param('track_width_m', 0.84).value,
                                 param('max_wheel_rpm', 94.7).value, param('turn_track_factor', 1.0).value)
        max_linear = param('max_linear_speed', -1.0).value  # <= 0: rover top speed from the wheel profile
        max_angular = param('max_angular_speed', -1.0).value  # <= 0: spin in place at full PWM
        config = TeleopConfig(
            axis_linear=param('axis_linear', defaults.axis_linear).value,
            axis_angular=param('axis_angular', defaults.axis_angular).value,
            invert_linear=param('invert_linear', defaults.invert_linear).value,
            invert_angular=param('invert_angular', defaults.invert_angular).value,
            deadzone=param('deadzone', defaults.deadzone).value,
            max_linear_speed=max_linear if max_linear > 0 else geometry.max_linear_speed,
            max_angular_speed=max_angular if max_angular > 0 else geometry.max_angular_speed,
            slow_button=param('slow_button', defaults.slow_button).value,
            slow_scale=param('slow_scale', defaults.slow_scale).value,
            estop_button=param('estop_button', defaults.estop_button).value,
            release_button=param('release_button', defaults.release_button).value,
            joy_timeout_s=param('joy_timeout_s', defaults.joy_timeout_s).value,
        )
        self.logic = TeleopLogic(config)
        self.cmd_vel = self.create_publisher(Twist, param('cmd_vel_topic', '/cmd_vel').value, LATEST_ONLY)
        self.estop = self.create_publisher(Bool, param('estop_topic', '/rover/estop').value, LATEST_ONLY)
        self.create_subscription(Joy, param('joy_topic', '/joy').value, self.on_joy, LATEST_ONLY)
        self.create_timer(1.0 / param('publish_rate_hz', 20.0).value, self.publish_cmd_vel)
        self.create_timer(1.0 / param('estop_rate_hz', 10.0).value, self.publish_estop)
        self.get_logger().info(
            f'teleop: full stick = {config.max_linear_speed:.2f} m/s, {config.max_angular_speed:.2f} rad/s; '
            f'e-stop button {config.estop_button}, release button {config.release_button} (sticks centred). '
            'E-stop is latched until released.')

    def on_joy(self, msg: Joy) -> None:
        for event in self.logic.on_joy(list(msg.axes), list(msg.buttons), time.monotonic()):
            self.get_logger().warn(event) if 'E-STOP' in event or 'ignored' in event else self.get_logger().info(event)

    def publish_cmd_vel(self) -> None:
        linear, angular, events = self.logic.tick(time.monotonic())
        for event in events:
            self.get_logger().warn(event)
        message = Twist()
        message.linear.x, message.angular.z = linear, angular
        self.cmd_vel.publish(message)

    def publish_estop(self) -> None:
        self.estop.publish(Bool(data=self.logic.estopped))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = BaseTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Leave the rover stopped: a final latched e-stop and a zero command.
        node.estop.publish(Bool(data=True))
        node.cmd_vel.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
