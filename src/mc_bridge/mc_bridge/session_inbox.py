"""Session-aware buffering for Mission Control input packets."""

from __future__ import annotations

from collections.abc import Callable
import queue
import threading
from typing import TypeAlias

from mc_bridge.packet import JsonObject


PacketHandler: TypeAlias = Callable[[JsonObject], None]
SessionPacket: TypeAlias = tuple[int, JsonObject]
PacketKey: TypeAlias = tuple[str, ...]
_IDENTITY_FIELDS = {
    'cameraFrameRequest': 'camera',
    'jointPositionRequest': 'joint',
    'servoPositionRequest': 'servo',
}


class SessionInbox:
    """Bound input while preventing packets from crossing sessions."""

    def __init__(self, event_capacity: int) -> None:
        """Create priority and keyed latest-value command slots."""
        if event_capacity < 1:
            raise ValueError('Event capacity must be positive')
        self._events: queue.Queue[SessionPacket] = queue.Queue(
            maxsize=event_capacity,
        )
        self._lock = threading.Lock()
        self._generation = 0
        self._active_generation: int | None = None
        self._pending_estop: SessionPacket | None = None
        self._uncommitted_stop: SessionPacket | None = None
        self._pending_motion: SessionPacket | None = None
        self._pending_joint_power: dict[str, SessionPacket] = {}
        self._pending_camera_stream: dict[str, SessionPacket] = {}
        self._pending_state: dict[PacketKey, SessionPacket] = {}

    def begin(self) -> None:
        """Open a fresh packet generation for a connected controller."""
        with self._lock:
            self._generation += 1
            self._active_generation = self._generation
            self._pending_estop = None
            self._uncommitted_stop = None
            self._pending_motion = None
            self._pending_joint_power.clear()
            self._pending_camera_stream.clear()
            self._pending_state.clear()
            self._discard_events()

    def end(self) -> bool:
        """Close the controller and report a stop that still needs latching."""
        with self._lock:
            pending_stop = (
                (
                    self._pending_estop is not None
                    and self._pending_estop[1].get('stop') is True
                )
                or self._uncommitted_stop is not None
            )
            self._active_generation = None
            self._pending_estop = None
            self._uncommitted_stop = None
            self._pending_motion = None
            self._pending_joint_power.clear()
            self._pending_camera_stream.clear()
            self._pending_state.clear()
            self._discard_events()
            return pending_stop

    def offer(self, packet: JsonObject) -> bool:
        """Retain a packet for the current controller without blocking."""
        snapshot = dict(packet)
        with self._lock:
            generation = self._active_generation
            if generation is None:
                return False
            session_packet = generation, snapshot
            packet_type = snapshot['type']
            if packet_type == 'emergencyStopRequest':
                pending_stop = self._pending_estop
                if pending_stop is None or snapshot.get('stop') is True:
                    self._pending_estop = session_packet
                return True
            if packet_type in ('driveRequest', 'tankDriveRequest'):
                self._pending_motion = session_packet
                return True
            if packet_type == 'jointPowerRequest':
                joint = snapshot.get('joint')
                if not isinstance(joint, str):
                    return False
                self._pending_joint_power[joint] = session_packet
                return True
            if packet_type in (
                'cameraStreamOpenRequest',
                'cameraStreamCloseRequest',
            ):
                camera = snapshot.get('camera')
                if not isinstance(camera, str):
                    return False
                self._pending_camera_stream[camera] = session_packet
                return True
            state_key = _state_key(snapshot)
            if state_key is not None:
                self._pending_state[state_key] = session_packet
                return True
            try:
                self._events.put_nowait(session_packet)
            except queue.Full:
                return False
            return True

    def take(self, event_limit: int) -> list[SessionPacket]:
        """Take safety controls, latest values, then bounded events."""
        if event_limit < 1:
            raise ValueError('Event limit must be positive')
        pending: list[SessionPacket] = []
        with self._lock:
            if self._pending_estop is not None:
                pending.append(self._pending_estop)
                stop_requested = self._pending_estop[1].get('stop') is True
                if stop_requested:
                    self._uncommitted_stop = self._pending_estop
                self._pending_estop = None
                if stop_requested:
                    self._pending_motion = None
                    self._pending_joint_power.clear()
            if self._pending_motion is not None:
                pending.append(self._pending_motion)
                self._pending_motion = None
            pending.extend(self._pending_joint_power.values())
            self._pending_joint_power.clear()
            pending.extend(self._pending_camera_stream.values())
            self._pending_camera_stream.clear()
            pending.extend(self._pending_state.values())
            self._pending_state.clear()
            for _ in range(event_limit):
                try:
                    pending.append(self._events.get_nowait())
                except queue.Empty:
                    break
        return pending

    def dispatch(
        self,
        session_packet: SessionPacket,
        handler: PacketHandler,
    ) -> bool:
        """Handle a packet only while its originating session is active."""
        generation, packet = session_packet
        with self._lock:
            if generation != self._active_generation:
                return False
            handler(packet)
            if self._uncommitted_stop == session_packet:
                self._uncommitted_stop = None
            return True

    def _discard_events(self) -> None:
        while True:
            try:
                self._events.get_nowait()
            except queue.Empty:
                return


def _state_key(packet: JsonObject) -> PacketKey | None:
    packet_type = packet.get('type')
    if not isinstance(packet_type, str):
        return None
    identity_field = _IDENTITY_FIELDS.get(packet_type)
    if identity_field is not None:
        identity = packet.get(identity_field)
        if not isinstance(identity, str):
            return None
        return packet_type, identity
    if packet_type in (
        'armIKRequest',
        'operationModeRequest',
        'waypointNavRequest',
    ):
        return (packet_type,)
    return None
