"""Transport-neutral rover command watchdog and emergency-stop gate."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True, slots=True)
class WheelTargets:
    """Normalized left and right wheel targets."""

    left: float = 0.0
    right: float = 0.0


@dataclass(frozen=True, slots=True)
class JointPowerTarget:
    """One normalized, watchdog-controlled joint output."""

    joint: str
    power: float


@dataclass(frozen=True, slots=True)
class SafetySnapshot:
    """Observable output of the safety gate."""

    active_session_id: int
    emergency_stop_latched: bool
    heartbeat_timed_out: bool
    command_timed_out: bool
    wheels: WheelTargets
    joint_powers: tuple[JointPowerTarget, ...]


class SafetyGate:
    """Allow fresh motion commands only while a local heartbeat is healthy."""

    def __init__(
        self,
        heartbeat_timeout_sec: float,
        command_timeout_sec: float,
        *,
        emergency_stop_latched: bool = False,
    ) -> None:
        """Create a stopped gate with a timed-out watchdog."""
        if heartbeat_timeout_sec <= 0.0:
            raise ValueError('Heartbeat timeout must be positive')
        if command_timeout_sec <= 0.0:
            raise ValueError('Command timeout must be positive')
        if not isinstance(emergency_stop_latched, bool):
            raise TypeError('Emergency-stop latch state must be boolean')
        self._heartbeat_timeout = heartbeat_timeout_sec
        self._command_timeout = command_timeout_sec
        self._active_session_id = 0
        self._heartbeat_sequence = -1
        self._motion_sequence = -1
        self._estop_sequence = -1
        self._joint_sequences: dict[str, int] = {}
        self._command_sequences: dict[str, int] = {}
        self._session_started: float | None = None
        self._last_heartbeat: float | None = None
        self._last_motion: float | None = None
        self._last_joint_power: dict[str, float] = {}
        self._estop_latched = emergency_stop_latched
        self._heartbeat_timed_out = True
        self._command_timed_out = True
        self._wheels = WheelTargets()
        self._joint_powers: dict[str, float] = {}
        self._retired_sessions: deque[int] = deque(maxlen=32)

    @property
    def snapshot(self) -> SafetySnapshot:
        """Return the current safe output state."""
        return SafetySnapshot(
            active_session_id=self._active_session_id,
            emergency_stop_latched=self._estop_latched,
            heartbeat_timed_out=self._heartbeat_timed_out,
            command_timed_out=self._command_timed_out,
            wheels=self._wheels,
            joint_powers=tuple(
                JointPowerTarget(joint, power)
                for joint, power in sorted(self._joint_powers.items())
            ),
        )

    def begin_session(self, session_id: int, now: float) -> bool:
        """Activate a session only after its one-shot start message."""
        if session_id <= 0 or session_id in self._retired_sessions:
            return False
        if session_id == self._active_session_id:
            return False
        if self._active_session_id:
            self._retired_sessions.append(self._active_session_id)
        self._active_session_id = session_id
        self._heartbeat_sequence = -1
        self._motion_sequence = -1
        self._estop_sequence = -1
        self._joint_sequences.clear()
        self._command_sequences.clear()
        self._session_started = now
        self._last_heartbeat = None
        self._last_motion = None
        self._last_joint_power.clear()
        self._heartbeat_timed_out = True
        self._command_timed_out = True
        self._wheels = WheelTargets()
        self._joint_powers.clear()
        return True

    def receive_heartbeat(
        self,
        session_id: int,
        sequence: int,
        now: float,
    ) -> bool:
        """Refresh the watchdog with a fresh connection-scoped heartbeat."""
        if session_id <= 0 or session_id != self._active_session_id:
            return False
        heartbeat_reference = (
            self._last_heartbeat
            if self._last_heartbeat is not None
            else self._session_started
        )
        if (
            heartbeat_reference is None
            or now - heartbeat_reference >= self._heartbeat_timeout
        ):
            self._retire_active_session()
            return False
        if sequence <= self._heartbeat_sequence:
            return False
        self._heartbeat_sequence = sequence
        self._session_started = None
        self._last_heartbeat = now
        self._heartbeat_timed_out = False
        return True

    def accept_command(
        self,
        session_id: int,
        sequence: int,
        key: str,
    ) -> bool:
        """Consume an event and allow it only for the healthy session."""
        if not key:
            return False
        previous_sequence = self._command_sequences.get(key, -1)
        if (
            session_id != self._active_session_id
            or sequence <= previous_sequence
        ):
            return False
        self._command_sequences[key] = sequence
        return not self._heartbeat_timed_out and not self._estop_latched

    def receive_drive(
        self,
        session_id: int,
        sequence: int,
        straight: float,
        steer: float,
        now: float,
    ) -> bool:
        """Apply a fresh normalized arcade-drive command."""
        if (
            not _is_normalized(straight, steer)
            or not self._accept_motion(session_id, sequence)
        ):
            return False
        self._wheels = WheelTargets(
            left=_clamp(straight + steer),
            right=_clamp(straight - steer),
        )
        self._last_motion = now
        self._command_timed_out = False
        return True

    def receive_joint_power(
        self,
        session_id: int,
        sequence: int,
        joint: str,
        power: float,
        now: float,
    ) -> bool:
        """Apply a fresh normalized joint command for the healthy session."""
        if not joint or not _is_normalized(power):
            return False
        previous_sequence = self._joint_sequences.get(joint, -1)
        if (
            session_id != self._active_session_id
            or sequence <= previous_sequence
        ):
            return False
        self._joint_sequences[joint] = sequence
        if self._heartbeat_timed_out or self._estop_latched:
            return False
        if power == 0.0:
            self._joint_powers.pop(joint, None)
            self._last_joint_power.pop(joint, None)
        else:
            self._joint_powers[joint] = float(power)
            self._last_joint_power[joint] = now
        return True

    def receive_tank_drive(
        self,
        session_id: int,
        sequence: int,
        left: float,
        right: float,
        now: float,
    ) -> bool:
        """Apply a fresh normalized tank-drive command."""
        if (
            not _is_normalized(left, right)
            or not self._accept_motion(session_id, sequence)
        ):
            return False
        self._wheels = WheelTargets(_clamp(left), _clamp(right))
        self._last_motion = now
        self._command_timed_out = False
        return True

    def receive_emergency_stop(
        self,
        session_id: int,
        sequence: int,
        stop: bool,
    ) -> bool:
        """Latch any stop; accept a clear only from the healthy session."""
        if stop:
            self._estop_latched = True
            self._last_motion = None
            self._last_joint_power.clear()
            self._command_timed_out = True
            self._wheels = WheelTargets()
            self._joint_powers.clear()
            if session_id == self._active_session_id:
                self._estop_sequence = max(self._estop_sequence, sequence)
            return True
        if (
            session_id != self._active_session_id
            or sequence <= self._estop_sequence
        ):
            return False
        self._estop_sequence = sequence
        if self._heartbeat_timed_out:
            return False
        was_latched = self._estop_latched
        self._estop_latched = False
        if was_latched:
            self._last_motion = None
            self._last_joint_power.clear()
            self._command_timed_out = True
            self._wheels = WheelTargets()
            self._joint_powers.clear()
        return True

    def check_timeouts(self, now: float) -> bool:
        """Stop motion when heartbeat or command age reaches its timeout."""
        heartbeat_reference = (
            self._last_heartbeat
            if self._last_heartbeat is not None
            else self._session_started
        )
        heartbeat_timed_out = (
            heartbeat_reference is None
            or now - heartbeat_reference >= self._heartbeat_timeout
        )
        command_timed_out = (
            self._last_motion is None
            or now - self._last_motion >= self._command_timeout
        )
        expired_joints = {
            joint
            for joint, updated_at in self._last_joint_power.items()
            if now - updated_at >= self._command_timeout
        }
        changed = False
        if heartbeat_timed_out and self._active_session_id:
            self._retire_active_session()
            changed = True
        elif heartbeat_timed_out and not self._heartbeat_timed_out:
            self._heartbeat_timed_out = True
            changed = True
        if command_timed_out and not self._command_timed_out:
            self._command_timed_out = True
            changed = True
        if heartbeat_timed_out:
            expired_joints.update(self._joint_powers)
        if expired_joints:
            for joint in expired_joints:
                self._joint_powers.pop(joint, None)
                self._last_joint_power.pop(joint, None)
            changed = True
        if heartbeat_timed_out or command_timed_out:
            self._wheels = WheelTargets()
        return changed

    def _accept_motion(self, session_id: int, sequence: int) -> bool:
        if (
            session_id != self._active_session_id
            or sequence <= self._motion_sequence
        ):
            return False
        self._motion_sequence = sequence
        return not self._heartbeat_timed_out and not self._estop_latched

    def _retire_active_session(self) -> None:
        if self._active_session_id:
            self._retired_sessions.append(self._active_session_id)
        self._active_session_id = 0
        self._session_started = None
        self._last_heartbeat = None
        self._last_motion = None
        self._last_joint_power.clear()
        self._heartbeat_timed_out = True
        self._command_timed_out = True
        self._wheels = WheelTargets()
        self._joint_powers.clear()


def _clamp(value: float) -> float:
    return max(-1.0, min(1.0, float(value)))


def _is_normalized(*values: float) -> bool:
    return all(
        math.isfinite(value) and -1.0 <= value <= 1.0
        for value in values
    )
