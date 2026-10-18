import pytest

from rover_pilot_remastered.teleop_logic import TeleopConfig, TeleopLogic, apply_deadzone

CIRCLE, OPTIONS, L1 = 1, 6, 9


def joy(left_y=0.0, right_x=0.0, pressed=()):
    axes = [0.0, left_y, right_x, 0.0, 1.0, 1.0]
    buttons = [1 if i in pressed else 0 for i in range(15)]
    return axes, buttons


def released(logic, now=0.0):
    logic.on_joy(*joy(pressed=(OPTIONS,)), now)
    logic.on_joy(*joy(), now)
    assert not logic.estopped


def test_starts_latched_and_needs_release_with_centred_sticks():
    logic = TeleopLogic(TeleopConfig())
    logic.on_joy(*joy(left_y=1.0), 0.0)
    assert logic.tick(0.0)[:2] == (0.0, 0.0) and logic.estopped
    events = logic.on_joy(*joy(left_y=1.0, pressed=(OPTIONS,)), 0.1)
    assert logic.estopped and 'centre' in events[0]
    logic.on_joy(*joy(), 0.2)
    logic.on_joy(*joy(pressed=(OPTIONS,)), 0.3)
    assert not logic.estopped


def test_proportional_output_with_deadzone_and_slow_mode():
    config = TeleopConfig(max_linear_speed=1.389, max_angular_speed=3.307)
    logic = TeleopLogic(config)
    released(logic)
    logic.on_joy(*joy(left_y=1.0, right_x=-0.5), 0.1)
    linear, angular, _ = logic.tick(0.1)
    assert linear == pytest.approx(1.389)
    assert angular == pytest.approx(-apply_deadzone(0.5, 0.08) * 3.307)  # stick right -> turn right (negative yaw)
    logic.on_joy(*joy(left_y=1.0, pressed=(L1,)), 0.2)
    assert logic.tick(0.2)[0] == pytest.approx(1.389 * 0.35)
    logic.on_joy(*joy(left_y=0.05), 0.3)
    assert logic.tick(0.3)[0] == 0.0


def test_estop_button_latches_and_release_needs_a_new_press():
    logic = TeleopLogic(TeleopConfig())
    released(logic)
    logic.on_joy(*joy(left_y=0.8, pressed=(CIRCLE,)), 1.0)
    assert logic.estopped and logic.tick(1.0)[:2] == (0.0, 0.0)
    logic.on_joy(*joy(pressed=(CIRCLE, OPTIONS)), 1.1)  # release ignored while the e-stop button is held
    assert logic.estopped
    logic.on_joy(*joy(pressed=(OPTIONS,)), 1.2)  # still held from before: not a new press
    assert logic.estopped
    logic.on_joy(*joy(), 1.3)
    logic.on_joy(*joy(pressed=(OPTIONS,)), 1.4)
    assert not logic.estopped


def test_joystick_loss_latches():
    logic = TeleopLogic(TeleopConfig(joy_timeout_s=0.5))
    released(logic)
    logic.on_joy(*joy(left_y=1.0), 1.0)
    assert logic.tick(1.4)[0] > 0
    linear, _, events = logic.tick(1.6)
    assert linear == 0.0 and logic.estopped and 'joystick lost' in events[0]
    logic.on_joy(*joy(left_y=1.0), 2.0)  # joystick back with the stick pushed: stays stopped
    assert logic.tick(2.0)[0] == 0.0 and logic.estopped


def test_short_or_missing_axes_are_safe():
    logic = TeleopLogic(TeleopConfig(axis_linear=7))
    released(logic)
    assert logic.tick(0.1)[:2] == (0.0, 0.0)
