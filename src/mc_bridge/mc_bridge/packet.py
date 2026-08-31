"""Shared JSON packet-envelope encoding and validation."""

from __future__ import annotations

import json
import math
from typing import TypeAlias, cast


JsonObject: TypeAlias = dict[str, object]


class PacketValidationError(ValueError):
    """Report a malformed Mission Control packet."""


def decode_packet(message: str) -> JsonObject:
    """Decode a strict JSON object with a non-empty packet type."""
    try:
        packet = json.loads(
            message,
            parse_constant=_reject_json_constant,
            parse_float=_parse_finite_float,
        )
    except (json.JSONDecodeError, ValueError) as error:
        raise PacketValidationError(
            'Packet must contain valid JSON',
        ) from error
    return _validate_envelope(packet)


def encode_packet(packet: JsonObject) -> str:
    """Encode a packet as compact, standards-compliant JSON."""
    _validate_envelope(packet)
    try:
        return json.dumps(packet, allow_nan=False, separators=(',', ':'))
    except (TypeError, ValueError) as error:
        raise PacketValidationError(
            'Packet contains unsupported data',
        ) from error


def validate_normalized_fields(
    packet: JsonObject,
    *field_names: str,
) -> None:
    """Require finite numeric fields in the inclusive range [-1, 1]."""
    for field_name in field_names:
        value = packet.get(field_name)
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not -1.0 <= value <= 1.0
        ):
            raise PacketValidationError(
                f'{field_name} must be a finite number in [-1, 1]',
            )


def _validate_envelope(packet: object) -> JsonObject:
    if not isinstance(packet, dict):
        raise PacketValidationError('Packet must be a JSON object')
    packet_type = packet.get('type')
    if not isinstance(packet_type, str) or not packet_type.strip():
        raise PacketValidationError('Packet type must be a non-empty string')
    return cast(JsonObject, packet)


def _reject_json_constant(value: str) -> object:
    raise ValueError(f'Invalid JSON constant: {value}')


def _parse_finite_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(
            f'JSON number is outside the supported range: {value}',
        )
    return parsed
