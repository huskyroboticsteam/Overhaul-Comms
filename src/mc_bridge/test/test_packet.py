"""Tests for the shared Mission Control packet contract."""

import json
from pathlib import Path

import pytest

from mc_bridge.packet import (
    PacketValidationError,
    decode_packet,
    encode_packet,
    validate_packet,
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


@pytest.mark.parametrize('value', [True, None, 1.01, -1.01])
def test_drive_range_is_validated(value: object) -> None:
    """The canonical contract enforces normalized drive values."""
    packet: dict[str, object] = {
        'type': 'driveRequest',
        'straight': value,
        'steer': 0.0,
    }
    with pytest.raises(PacketValidationError):
        validate_packet(packet, direction='request')


def test_normalized_range_includes_endpoints() -> None:
    """Both endpoints of the normalized range are valid."""
    packet: dict[str, object] = {
        'type': 'driveRequest',
        'straight': -1,
        'steer': 1.0,
    }
    validate_packet(packet, direction='request')


def test_packet_direction_is_enforced() -> None:
    """Mission Control cannot inject a rover report as a request."""
    packet: dict[str, object] = {
        'type': 'mountedPeripheralReport',
        'peripheral': 'arm',
    }
    with pytest.raises(PacketValidationError):
        validate_packet(packet, direction='request')


def test_vertical_slice_fixtures_match_contract() -> None:
    """Representative vertical-slice examples remain compatible with v1."""
    fixture_path = (
        Path(__file__).resolve().parents[3]
        / 'protocol'
        / 'fixtures'
        / 'vertical_slice.json'
    )
    packets = json.loads(fixture_path.read_text(encoding='utf-8'))
    for packet in packets:
        validate_packet(packet, direction='request')


def test_implemented_packet_fixtures_match_contract() -> None:
    """Every routed request and report remains compatible with v1."""
    fixture_path = (
        Path(__file__).resolve().parents[3]
        / 'protocol'
        / 'fixtures'
        / 'implemented_packets.json'
    )
    fixtures = json.loads(fixture_path.read_text(encoding='utf-8'))
    for direction in ('request', 'report'):
        for packet in fixtures[f'{direction}s']:
            validate_packet(packet, direction=direction)


def test_fixtures_cover_every_supported_packet_type() -> None:
    """The executable fixture set cannot drift behind the v1 contract."""
    root = Path(__file__).resolve().parents[3]
    schema = json.loads(
        (root / 'protocol' / 'packet.schema.json').read_text(
            encoding='utf-8',
        ),
    )
    fixtures = json.loads(
        (root / 'protocol' / 'fixtures' / 'implemented_packets.json')
        .read_text(encoding='utf-8'),
    )
    for direction in ('request', 'report'):
        supported = {
            definition['properties']['type']['const']
            for definition in schema['$defs'].values()
            if definition.get('x-direction') == direction
            and definition.get('x-bridge-support') == 'active'
        }
        represented = {
            packet['type']
            for packet in fixtures[f'{direction}s']
        }
        assert represented == supported


def test_every_packet_definition_has_compatibility_metadata() -> None:
    """Every packet in the union declares its direction and status."""
    schema_path = (
        Path(__file__).resolve().parents[3]
        / 'protocol'
        / 'packet.schema.json'
    )
    schema = json.loads(schema_path.read_text(encoding='utf-8'))
    packet_definitions = [
        definition
        for definition in schema['$defs'].values()
        if 'properties' in definition
    ]
    assert packet_definitions
    assert all(
        definition['x-direction'] in {'request', 'report'}
        for definition in packet_definitions
    )
    assert all(
        definition['x-status']
        in {'active', 'legacy-compatible', 'planned', 'unsupported'}
        for definition in packet_definitions
    )
    assert all(
        definition['x-bridge-support'] == 'active'
        for definition in packet_definitions
    )
