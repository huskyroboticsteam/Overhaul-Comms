"""Session-aware buffering for Mission Control input packets."""

from __future__ import annotations

from collections.abc import Callable
import queue
import threading
from typing import TypeAlias

from mc_bridge.packet import JsonObject


PacketHandler: TypeAlias = Callable[[JsonObject], None]
SessionPacket: TypeAlias = tuple[int, JsonObject]


class SessionInbox:
    """Bound input while preventing packets from crossing sessions."""

    def __init__(self, event_capacity: int) -> None:
        """Create an inbox with priority and latest-motion slots."""
        if event_capacity < 1:
            raise ValueError('Event capacity must be positive')
        self._events: queue.Queue[SessionPacket] = queue.Queue(
            maxsize=event_capacity,
        )
        self._lock = threading.Lock()
        self._generation = 0
        self._active_generation: int | None = None
        self._pending_estop: SessionPacket | None = None
        self._pending_motion: SessionPacket | None = None

    def begin(self) -> None:
        """Open a fresh packet generation for a connected controller."""
        with self._lock:
            self._generation += 1
            self._active_generation = self._generation
            self._pending_estop = None
            self._pending_motion = None
            self._discard_events()

    def end(self) -> None:
        """Invalidate the controller and discard all of its pending input."""
        with self._lock:
            self._active_generation = None
            self._pending_estop = None
            self._pending_motion = None
            self._discard_events()

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
            try:
                self._events.put_nowait(session_packet)
            except queue.Full:
                return False
            return True

    def take(self, event_limit: int) -> list[SessionPacket]:
        """Take e-stop first, then latest motion and a bounded event batch."""
        if event_limit < 1:
            raise ValueError('Event limit must be positive')
        pending: list[SessionPacket] = []
        with self._lock:
            if self._pending_estop is not None:
                pending.append(self._pending_estop)
                stop_requested = self._pending_estop[1].get('stop') is True
                self._pending_estop = None
                if stop_requested:
                    self._pending_motion = None
            if self._pending_motion is not None:
                pending.append(self._pending_motion)
                self._pending_motion = None
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
            return True

    def _discard_events(self) -> None:
        while True:
            try:
                self._events.get_nowait()
            except queue.Empty:
                return
