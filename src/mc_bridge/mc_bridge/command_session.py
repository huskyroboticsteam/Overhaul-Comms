"""Thread-safe connection identity and command sequencing."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import secrets
import threading


@dataclass(frozen=True, slots=True)
class CommandStamp:
    """Identity shared by one command and its WebSocket session."""

    session_id: int
    sequence: int


class CommandSession:
    """Issue monotonically ordered stamps for the active controller."""

    def __init__(
        self,
        session_id_factory: Callable[[], int] | None = None,
    ) -> None:
        """Create an inactive sequencer."""
        self._session_id_factory = session_id_factory or _random_session_id
        self._lock = threading.Lock()
        self._session_id = 0
        self._sequence = 0

    def begin(self) -> int:
        """Start a new nonzero session and return its identifier."""
        with self._lock:
            session_id = self._session_id_factory()
            if not 0 < session_id < 2**64:
                raise ValueError('Session ID must be a nonzero uint64')
            self._session_id = session_id
            self._sequence = 0
            return session_id

    @property
    def active_session_id(self) -> int:
        """Return the active session identifier, or zero when disconnected."""
        with self._lock:
            return self._session_id

    def next_stamp(self) -> CommandStamp | None:
        """Return the next stamp, or None when no controller is active."""
        with self._lock:
            return self._next_stamp()

    def end(self) -> CommandStamp | None:
        """Close the session and return a final neutral-command stamp."""
        with self._lock:
            stamp = self._next_stamp()
            self._session_id = 0
            self._sequence = 0
            return stamp

    def _next_stamp(self) -> CommandStamp | None:
        if not self._session_id:
            return None
        self._sequence += 1
        return CommandStamp(self._session_id, self._sequence)


def _random_session_id() -> int:
    session_id = 0
    while not session_id:
        session_id = secrets.randbits(64)
    return session_id
