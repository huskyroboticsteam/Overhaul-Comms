"""Tests for connection-scoped command metadata."""

from mc_bridge.command_session import CommandSession, CommandStamp


def test_stamps_are_ordered_within_a_session() -> None:
    """Every command in one connection receives a newer sequence."""
    session = CommandSession(lambda: 42)
    assert session.begin() == 42
    assert session.next_stamp() == CommandStamp(42, 1)
    assert session.next_stamp() == CommandStamp(42, 2)


def test_disconnect_issues_final_stamp_and_invalidates_session() -> None:
    """Disconnect reserves a final neutral command before deactivation."""
    session = CommandSession(lambda: 42)
    session.begin()
    assert session.end() == CommandStamp(42, 1)
    assert session.next_stamp() is None
