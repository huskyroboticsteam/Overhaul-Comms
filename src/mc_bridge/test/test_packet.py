"""Tests for the shared Mission Control packet envelope."""

import math

import pytest

from mc_bridge.packet import (
    PacketValidationError,
    decode_packet,
    encode_packet,
    validate_normalized_fields,
)


@pytest.mark.parametrize(
    'message',
    [
        'not-json',
        'null',
        '[]',
        '{}',
        '{"type":""}',
        '{"type":"state","value":NaN}',
        '{"type":"state","value":1e999}',
    ],
)
def test_decode_packet_rejects_invalid_envelopes(message: str) -> None:
    """Only strict JSON objects with a packet type are accepted."""
    with pytest.raises(PacketValidationError):
        decode_packet(message)


def test_packet_round_trip() -> None:
    """Valid packets use compact JSON without changing their data."""
    packet: dict[str, object] = {
        'type': 'driveRequest',
        'straight': 0.5,
        'steer': -0.25,
    }
    assert decode_packet(encode_packet(packet)) == packet


@pytest.mark.parametrize('value', [True, None, 1.01, -1.01, math.nan])
def test_normalized_fields_are_validated(value: object) -> None:
    """Drive values must be finite numbers in the normalized range."""
    packet: dict[str, object] = {'type': 'driveRequest', 'straight': value}
    with pytest.raises(PacketValidationError):
        validate_normalized_fields(packet, 'straight')


def test_normalized_range_includes_endpoints() -> None:
    """Both endpoints of the normalized range are valid."""
    packet: dict[str, object] = {
        'type': 'driveRequest',
        'straight': -1,
        'steer': 1.0,
    }
    validate_normalized_fields(packet, 'straight', 'steer')
