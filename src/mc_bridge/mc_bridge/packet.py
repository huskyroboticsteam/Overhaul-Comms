"""Shared JSON packet-envelope encoding and validation."""

from __future__ import annotations

import json
from functools import lru_cache
import math
from pathlib import Path
from typing import TypeAlias, cast

from jsonschema import Draft202012Validator  # type: ignore[import-untyped]
from jsonschema.exceptions import best_match  # type: ignore[import-untyped]


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


def validate_packet(packet: JsonObject, direction: str | None = None) -> None:
    """Validate a packet against the canonical versioned contract."""
    validator, definitions = _contract()
    error = best_match(validator.iter_errors(packet))
    if error is not None:
        raise PacketValidationError(error.message)
    packet_type = cast(str, packet['type'])
    packet_direction = definitions[packet_type].get('x-direction')
    if direction is not None and packet_direction != direction:
        raise PacketValidationError(
            f'{packet_type} is not a {direction} packet',
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


@lru_cache(maxsize=1)
def _contract() -> tuple[Draft202012Validator, dict[str, JsonObject]]:
    schema_path = _contract_path()
    try:
        schema = json.loads(schema_path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError(
            f'Could not load packet contract: {schema_path}',
        ) from error
    Draft202012Validator.check_schema(schema)
    definitions = {
        definition['properties']['type']['const']: definition
        for definition in schema['$defs'].values()
        if isinstance(definition, dict)
        and 'properties' in definition
        and 'type' in definition['properties']
        and 'const' in definition['properties']['type']
    }
    return Draft202012Validator(schema), definitions


def _contract_path() -> Path:
    source_path = Path(__file__).resolve().parents[3] / 'protocol' / (
        'packet.schema.json'
    )
    if source_path.is_file():
        return source_path

    from ament_index_python.packages import (  # type: ignore[import-not-found]
        get_package_share_directory,
    )

    return (
        Path(get_package_share_directory('mc_bridge'))
        / 'protocol'
        / 'packet.schema.json'
    )
