"""Tests for the transport-independent rover safety gate."""

import math

from rover_safety.safety_gate import (
    JointPowerTarget,
    SafetyGate,
    WheelTargets,
)


def _healthy_gate() -> SafetyGate:
    gate = SafetyGate(
        heartbeat_timeout_sec=0.5,
        command_timeout_sec=0.3,
    )
    assert gate.begin_session(10, now=0.9)
    assert gate.receive_heartbeat(session_id=10, sequence=1, now=1.0)
    return gate


def test_motion_requires_a_live_heartbeat() -> None:
    """Motion is rejected before the first session heartbeat."""
    gate = SafetyGate(heartbeat_timeout_sec=0.5, command_timeout_sec=0.3)
    assert gate.begin_session(10, now=0.9)
    assert not gate.receive_drive(10, 1, 1.0, 0.0, now=1.0)
    assert gate.snapshot.wheels == WheelTargets()


def test_heartbeat_cannot_create_a_session() -> None:
    """A watchdog restart cannot recover a still-running laptop lease."""
    gate = SafetyGate(heartbeat_timeout_sec=0.5, command_timeout_sec=0.3)
    assert not gate.receive_heartbeat(0, 1, now=0.9)
    assert not gate.receive_heartbeat(10, 1, now=1.0)
    assert not gate.receive_drive(10, 2, 1.0, 0.0, now=1.1)
    assert gate.snapshot.active_session_id == 0


def test_session_start_expires_without_a_heartbeat() -> None:
    """A delayed first heartbeat cannot revive an abandoned lease."""
    gate = SafetyGate(heartbeat_timeout_sec=0.5, command_timeout_sec=0.3)
    assert gate.begin_session(10, now=1.0)
    assert gate.check_timeouts(now=1.5)
    assert gate.snapshot.active_session_id == 0
    assert not gate.receive_heartbeat(10, 1, now=1.51)


def test_drive_and_tank_commands_produce_bounded_wheels() -> None:
    """Both supported drive modes produce normalized wheel targets."""
    gate = _healthy_gate()
    assert gate.receive_drive(10, 2, 0.75, 0.5, now=1.1)
    assert gate.snapshot.wheels == WheelTargets(1.0, 0.25)
    assert gate.receive_tank_drive(10, 3, -0.4, 0.6, now=1.2)
    assert gate.snapshot.wheels == WheelTargets(-0.4, 0.6)


def test_joint_power_is_keyed_and_watchdog_controlled() -> None:
    """Each joint retains only fresh power and expires independently."""
    gate = _healthy_gate()
    assert gate.receive_joint_power(10, 2, 'elbow', 0.5, now=1.1)
    assert gate.receive_joint_power(10, 3, 'wristPitch', -0.25, now=1.2)
    assert gate.snapshot.joint_powers == (
        JointPowerTarget('elbow', 0.5),
        JointPowerTarget('wristPitch', -0.25),
    )

    assert gate.check_timeouts(now=1.41)
    assert gate.snapshot.joint_powers == (
        JointPowerTarget('wristPitch', -0.25),
    )


def test_joint_power_neutralizes_on_estop_and_cannot_replay() -> None:
    """Commands observed during e-stop remain consumed after a clear."""
    gate = _healthy_gate()
    assert gate.receive_joint_power(10, 2, 'elbow', 0.5, now=1.1)
    assert gate.receive_emergency_stop(10, 3, stop=True)
    assert gate.snapshot.joint_powers == ()
    assert not gate.receive_joint_power(10, 4, 'elbow', 1.0, now=1.2)
    assert gate.receive_emergency_stop(10, 5, stop=False)
    assert not gate.receive_joint_power(10, 4, 'elbow', 1.0, now=1.3)
    assert gate.snapshot.joint_powers == ()


def test_event_commands_require_the_current_safety_lease() -> None:
    """Discrete commands are consumed while unsafe and cannot replay."""
    gate = _healthy_gate()
    assert gate.accept_command(10, 2, 'camera:mast')
    assert gate.receive_emergency_stop(10, 3, stop=True)
    assert not gate.accept_command(10, 4, 'camera:mast')
    assert gate.receive_emergency_stop(10, 5, stop=False)
    assert not gate.accept_command(10, 4, 'camera:mast')
    assert gate.accept_command(10, 6, 'camera:mast')


def test_watchdog_stops_motion_and_requires_a_new_command() -> None:
    """Heartbeat loss stops motion without replaying it after recovery."""
    gate = _healthy_gate()
    assert gate.receive_drive(10, 2, 1.0, 0.0, now=1.1)
    assert gate.check_timeouts(now=1.41)
    assert gate.snapshot.command_timed_out
    assert not gate.snapshot.heartbeat_timed_out
    assert gate.snapshot.wheels == WheelTargets()

    assert gate.receive_heartbeat(10, 3, now=1.45)
    assert not gate.snapshot.heartbeat_timed_out
    assert gate.snapshot.command_timed_out
    assert gate.snapshot.wheels == WheelTargets()


def test_heartbeat_timeout_requires_a_new_session() -> None:
    """Queued traffic from a failed radio lease cannot restart motion."""
    gate = _healthy_gate()
    assert gate.receive_drive(10, 2, 1.0, 0.0, now=1.1)

    assert gate.check_timeouts(now=1.5)
    assert gate.snapshot.active_session_id == 0
    assert gate.snapshot.heartbeat_timed_out
    assert not gate.receive_heartbeat(10, 99, now=1.51)
    assert not gate.receive_drive(10, 100, 1.0, 0.0, now=1.52)

    assert gate.begin_session(20, now=1.52)
    assert gate.receive_heartbeat(20, 1, now=1.53)
    assert gate.snapshot.active_session_id == 20
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


def test_persisted_emergency_stop_starts_latched() -> None:
    """A watchdog restart preserves the rover-local stop decision."""
    gate = SafetyGate(
        heartbeat_timeout_sec=0.5,
        command_timeout_sec=0.3,
        emergency_stop_latched=True,
    )
    assert gate.begin_session(10, now=0.9)
    assert gate.receive_heartbeat(10, 1, now=1.0)
    assert not gate.receive_drive(10, 2, 1.0, 0.0, now=1.1)
    assert gate.snapshot.emergency_stop_latched
    assert not gate.receive_emergency_stop(99, 2, stop=False)
    assert gate.receive_emergency_stop(10, 3, stop=False)
    assert not gate.snapshot.emergency_stop_latched
    assert gate.snapshot.wheels == WheelTargets()


def test_retired_session_packets_are_never_replayed() -> None:
    """A delayed heartbeat cannot reactivate a replaced session."""
    gate = _healthy_gate()
    assert gate.begin_session(20, now=1.05)
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
