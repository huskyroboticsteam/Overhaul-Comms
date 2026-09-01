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


def test_drive_modes_share_one_latest_motion_slot() -> None:
    """A newer tank command replaces an unhandled arcade command."""
    inbox = SessionInbox(event_capacity=2)
    inbox.begin()
    assert inbox.offer(
        {'type': 'driveRequest', 'straight': 1.0, 'steer': 0.0},
    )
    assert inbox.offer(
        {'type': 'tankDriveRequest', 'left': -1.0, 'right': 1.0},
    )
    assert inbox.take(event_limit=1)[0][1]['type'] == 'tankDriveRequest'


def test_joint_power_is_latest_value_per_joint() -> None:
    """Continuous arm controls coalesce without hiding other joints."""
    inbox = SessionInbox(event_capacity=1)
    inbox.begin()
    assert inbox.offer(
        {'type': 'jointPowerRequest', 'joint': 'elbow', 'power': 0.25},
    )
    assert inbox.offer(
        {'type': 'jointPowerRequest', 'joint': 'elbow', 'power': 0.75},
    )
    assert inbox.offer(
        {'type': 'jointPowerRequest', 'joint': 'wristPitch', 'power': -0.5},
    )

    packets = [packet for _, packet in inbox.take(event_limit=1)]
    assert packets == [
        {'type': 'jointPowerRequest', 'joint': 'elbow', 'power': 0.75},
        {
            'type': 'jointPowerRequest',
            'joint': 'wristPitch',
            'power': -0.5,
        },
    ]


def test_camera_stream_state_is_latest_value_per_camera() -> None:
    """Rapid stream toggles cannot create stale camera work."""
    inbox = SessionInbox(event_capacity=1)
    inbox.begin()
    assert inbox.offer(
        {'type': 'cameraStreamOpenRequest', 'camera': 'mast', 'fps': 20},
    )
    assert inbox.offer(
        {'type': 'cameraStreamOpenRequest', 'camera': 'mast', 'fps': 30},
    )
    assert inbox.offer(
        {'type': 'cameraStreamCloseRequest', 'camera': 'mast'},
    )
    assert inbox.offer(
        {'type': 'cameraStreamOpenRequest', 'camera': 'hand', 'fps': 10},
    )

    packets = [packet for _, packet in inbox.take(event_limit=1)]
    assert packets == [
        {'type': 'cameraStreamCloseRequest', 'camera': 'mast'},
        {'type': 'cameraStreamOpenRequest', 'camera': 'hand', 'fps': 10},
    ]


def test_state_commands_coalesce_by_identity() -> None:
    """Only current absolute targets survive a stalled ROS executor."""
    inbox = SessionInbox(event_capacity=1)
    inbox.begin()
    assert inbox.offer(
        {
            'type': 'jointPositionRequest',
            'joint': 'elbow',
            'position': 10.0,
        },
    )
    assert inbox.offer(
        {
            'type': 'jointPositionRequest',
            'joint': 'elbow',
            'position': 20.0,
        },
    )
    assert inbox.offer(
        {
            'type': 'jointPositionRequest',
            'joint': 'shoulder',
            'position': 30.0,
        },
    )

    packets = [packet for _, packet in inbox.take(event_limit=1)]
    assert [packet['position'] for packet in packets] == [20.0, 30.0]


def test_emergency_stop_has_priority() -> None:
    """A pending stop cannot be coalesced away and discards pending motion."""
    inbox = SessionInbox(event_capacity=1)
    inbox.begin()
    assert inbox.offer({'type': 'emergencyStopRequest', 'stop': True})
    assert inbox.offer({'type': 'emergencyStopRequest', 'stop': False})
    assert inbox.offer(
        {'type': 'driveRequest', 'straight': 1.0, 'steer': 0.0},
    )

    packets = inbox.take(event_limit=1)
    assert [packet[1]['type'] for packet in packets] == [
        'emergencyStopRequest',
    ]
    assert packets[0][1]['stop'] is True


def test_emergency_stop_discards_pending_joint_power() -> None:
    """A queued actuator command cannot survive a pending stop."""
    inbox = SessionInbox(event_capacity=1)
    inbox.begin()
    assert inbox.offer(
        {'type': 'jointPowerRequest', 'joint': 'elbow', 'power': 1.0},
    )
    assert inbox.offer({'type': 'emergencyStopRequest', 'stop': True})
    assert [packet['type'] for _, packet in inbox.take(event_limit=1)] == [
        'emergencyStopRequest',
    ]


def test_disconnect_preserves_an_accepted_emergency_stop() -> None:
    """A stop cannot be lost when its WebSocket closes before ROS polls."""
    inbox = SessionInbox(event_capacity=1)
    inbox.begin()
    assert inbox.offer({'type': 'emergencyStopRequest', 'stop': True})
    assert inbox.end()


def test_disconnect_preserves_a_taken_emergency_stop() -> None:
    """A stop remains committed while waiting for ROS dispatch."""
    inbox = SessionInbox(event_capacity=1)
    inbox.begin()
    assert inbox.offer({'type': 'emergencyStopRequest', 'stop': True})
    stop = inbox.take(event_limit=1)[0]

    assert inbox.end()
    assert not inbox.dispatch(stop, lambda _: None)


def test_disconnect_does_not_apply_a_pending_estop_clear() -> None:
    """A disconnected controller cannot clear the rover-side latch."""
    inbox = SessionInbox(event_capacity=1)
    inbox.begin()
    assert inbox.offer({'type': 'emergencyStopRequest', 'stop': False})
    assert not inbox.end()


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
