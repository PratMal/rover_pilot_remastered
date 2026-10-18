from rover_pilot_remastered.drive_logic import DriveGate


def test_waits_for_heartbeat_then_drives():
    gate = DriveGate()
    gate.on_cmd_vel(1.0, 0.2, 0.0)
    assert gate.decide(0.0).estop
    gate.on_estop(False, 0.0)
    decision = gate.decide(0.1)
    assert (decision.estop, decision.linear_x, decision.angular_z, decision.reason) == (False, 1.0, 0.2, '')


def test_estop_and_heartbeat_loss_send_the_estop_flag():
    gate = DriveGate(estop_timeout_s=0.5)
    gate.on_estop(True, 0.0)
    gate.on_cmd_vel(1.0, 0.0, 0.0)
    assert gate.decide(0.1).estop and gate.decide(0.1).reason == 'e-stop'
    gate.on_estop(False, 1.0)
    gate.on_cmd_vel(1.0, 0.0, 1.0)
    assert not gate.decide(1.1).estop
    decision = gate.decide(1.6)
    assert decision.estop and 'lost' in decision.reason


def test_stale_cmd_vel_gives_zero_without_estop():
    gate = DriveGate(cmd_timeout_s=0.3)
    gate.on_estop(False, 0.0)
    gate.on_cmd_vel(1.0, 0.0, 0.0)
    gate.on_estop(False, 0.4)
    decision = gate.decide(0.4)
    assert (decision.estop, decision.linear_x) == (False, 0.0)


def test_heartbeat_can_be_disabled_for_autonomy():
    gate = DriveGate(require_estop_heartbeat=False)
    gate.on_cmd_vel(0.5, 0.0, 0.0)
    assert gate.decide(0.1).linear_x == 0.5
