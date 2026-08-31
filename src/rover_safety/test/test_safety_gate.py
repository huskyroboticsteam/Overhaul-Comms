"""Tests for the transport-independent rover safety gate."""

import math

from rover_safety.safety_gate import SafetyGate, WheelTargets


def _healthy_gate() -> SafetyGate:
    gate = SafetyGate(
        heartbeat_timeout_sec=0.5,
        command_timeout_sec=0.3,
    )
    assert gate.receive_heartbeat(session_id=10, sequence=1, now=1.0)
    return gate


def test_motion_requires_a_live_heartbeat() -> None:
    """Motion is rejected before the first session heartbeat."""
    gate = SafetyGate(heartbeat_timeout_sec=0.5, command_timeout_sec=0.3)
    assert not gate.receive_drive(10, 1, 1.0, 0.0, now=1.0)
    assert gate.snapshot.wheels == WheelTargets()


def test_drive_and_tank_commands_produce_bounded_wheels() -> None:
    """Both supported drive modes produce normalized wheel targets."""
    gate = _healthy_gate()
    assert gate.receive_drive(10, 2, 0.75, 0.5, now=1.1)
    assert gate.snapshot.wheels == WheelTargets(1.0, 0.25)
    assert gate.receive_tank_drive(10, 3, -0.4, 0.6, now=1.2)
    assert gate.snapshot.wheels == WheelTargets(-0.4, 0.6)


def test_watchdog_stops_motion_and_requires_a_new_command() -> None:
    """Heartbeat loss stops motion without replaying it after recovery."""
    gate = _healthy_gate()
    assert gate.receive_drive(10, 2, 1.0, 0.0, now=1.1)
    assert gate.check_timeouts(now=1.41)
    assert gate.snapshot.command_timed_out
    assert not gate.snapshot.heartbeat_timed_out
    assert gate.snapshot.wheels == WheelTargets()

    assert gate.receive_heartbeat(10, 3, now=1.6)
    assert not gate.snapshot.heartbeat_timed_out
    assert gate.snapshot.command_timed_out
    assert gate.snapshot.wheels == WheelTargets()


def test_emergency_stop_is_latched_and_clear_does_not_resume_motion() -> None:
    """Only an explicit fresh clear unlocks the gate, still at neutral."""
    gate = _healthy_gate()
    assert gate.receive_drive(10, 2, 0.8, 0.0, now=1.1)
    assert gate.receive_emergency_stop(99, 1, stop=True)
    assert gate.snapshot.emergency_stop_latched
    assert gate.snapshot.wheels == WheelTargets()
    assert not gate.receive_emergency_stop(99, 2, stop=False)
    assert gate.receive_emergency_stop(10, 3, stop=False)
    assert not gate.snapshot.emergency_stop_latched
    assert gate.snapshot.wheels == WheelTargets()


def test_retired_session_packets_are_never_replayed() -> None:
    """A delayed heartbeat cannot reactivate a replaced session."""
    gate = _healthy_gate()
    assert gate.receive_heartbeat(20, 1, now=1.1)
    assert not gate.receive_heartbeat(10, 99, now=1.2)
    assert not gate.receive_drive(10, 100, 1.0, 0.0, now=1.3)
    assert gate.snapshot.active_session_id == 20


def test_nonfinite_local_commands_are_rejected() -> None:
    """A local ROS publisher cannot bypass numeric safety checks."""
    gate = _healthy_gate()
    assert not gate.receive_drive(10, 2, math.nan, 0.0, now=1.1)
    assert gate.snapshot.wheels == WheelTargets()


def test_rejected_motion_cannot_replay_after_estop_clear() -> None:
    """A command observed during e-stop remains consumed after clear."""
    gate = _healthy_gate()
    assert gate.receive_emergency_stop(10, 2, stop=True)
    assert not gate.receive_drive(10, 3, 1.0, 0.0, now=1.1)
    assert gate.receive_emergency_stop(10, 4, stop=False)
    assert not gate.receive_drive(10, 3, 1.0, 0.0, now=1.2)
    assert gate.snapshot.wheels == WheelTargets()


def test_repeated_clear_state_does_not_interrupt_fresh_motion() -> None:
    """Refreshing an already-clear e-stop state leaves fresh motion intact."""
    gate = _healthy_gate()
    assert gate.receive_emergency_stop(10, 2, stop=False)
    assert gate.receive_drive(10, 3, 0.5, 0.0, now=1.1)
    assert gate.receive_emergency_stop(10, 4, stop=False)
    assert gate.snapshot.wheels == WheelTargets(0.5, 0.5)
