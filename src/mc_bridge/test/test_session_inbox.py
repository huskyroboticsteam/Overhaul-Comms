"""Tests for session-aware Mission Control input buffering."""

import threading

from mc_bridge.packet import JsonObject
from mc_bridge.session_inbox import SessionInbox


def test_drive_requests_are_latest_value() -> None:
    """Only the newest unhandled drive request is retained."""
    inbox = SessionInbox(event_capacity=2)
    inbox.begin()
    for straight in (-1.0, 0.0, 1.0):
        assert inbox.offer(
            {
                'type': 'driveRequest',
                'straight': straight,
                'steer': 0.0,
            },
        )

    handled: list[JsonObject] = []
    pending = inbox.take(event_limit=2)
    assert len(pending) == 1
    assert inbox.dispatch(pending[0], handled.append)
    assert handled[0]['straight'] == 1.0


def test_closed_session_packets_cannot_reach_replacement() -> None:
    """A packet already taken from an old generation is still rejected."""
    inbox = SessionInbox(event_capacity=2)
    inbox.begin()
    assert inbox.offer({'type': 'cameraStreamCloseRequest', 'camera': 'mast'})
    stale_packet = inbox.take(event_limit=1)[0]

    inbox.end()
    inbox.begin()

    handled: list[JsonObject] = []
    assert not inbox.dispatch(stale_packet, handled.append)
    assert handled == []


def test_session_end_waits_for_inflight_dispatch() -> None:
    """Disconnect cannot finish while an accepted packet is publishing."""
    inbox = SessionInbox(event_capacity=2)
    inbox.begin()
    assert inbox.offer({'type': 'cameraStreamCloseRequest', 'camera': 'mast'})
    packet = inbox.take(event_limit=1)[0]
    handler_started = threading.Event()
    release_handler = threading.Event()
    end_started = threading.Event()
    session_ended = threading.Event()

    def blocking_handler(_: JsonObject) -> None:
        handler_started.set()
        assert release_handler.wait(timeout=1.0)

    def end_session() -> None:
        end_started.set()
        inbox.end()
        session_ended.set()

    dispatch_thread = threading.Thread(
        target=inbox.dispatch,
        args=(packet, blocking_handler),
    )
    end_thread = threading.Thread(
        target=end_session,
    )

    dispatch_thread.start()
    assert handler_started.wait(timeout=1.0)
    end_thread.start()
    try:
        assert end_started.wait(timeout=1.0)
        assert not session_ended.wait(timeout=0.05)
    finally:
        release_handler.set()
    dispatch_thread.join(timeout=1.0)
    end_thread.join(timeout=1.0)
    assert not dispatch_thread.is_alive()
    assert not end_thread.is_alive()
    assert session_ended.is_set()
