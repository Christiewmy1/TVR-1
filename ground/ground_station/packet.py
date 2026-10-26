"""Telemetry packet decoder: the ground-side mirror of firmware/src/telemetry.c.

The format is specified in docs/packet_spec.md. decode() turns one packet's
text into a Packet, or raises PacketError with a short `reason` code so the
caller can count dropped packets by cause.
"""

from __future__ import annotations

import binascii
import re
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional

VERSION = 1
MAX_LEN = 54  # TELEMETRY_MAX_LEN in telemetry.h

# Scaled field ranges (same as telemetry.h).
ALT_DM_RANGE = (-9999, 99999)
ANGLE_DD_RANGE = (-1800, 1800)
BATT_MV_RANGE = (0, 65535)

_FRAME = re.compile(r"\$([^*]*)\*([0-9A-Fa-f]{4})")
_INT = re.compile(r"-?[0-9]+")


class FlightState(IntEnum):
    """Same numbers as the FlightState enum in flight_state.h."""

    PAD = 0
    BOOST = 1
    COAST = 2
    APOGEE = 3
    DESCENT = 4
    LANDED = 5


@dataclass(frozen=True)
class Packet:
    seq: int
    time_ms: int
    state: FlightState
    altitude_m: Optional[float]  # None = the rocket had no valid reading
    pitch_deg: Optional[float]
    yaw_deg: Optional[float]
    battery_v: Optional[float]


class PacketError(ValueError):
    """A rejected packet. `reason` is one of: too_long, framing, checksum,
    field_count, version, bad_field."""

    def __init__(self, reason: str, detail: str) -> None:
        super().__init__(f"{reason}: {detail}")
        self.reason = reason


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE, the packet checksum (same as telemetry_crc16)."""
    return binascii.crc_hqx(data, 0xFFFF)


def _int_field(text: str, name: str, low: int, high: int, required: bool) -> Optional[int]:
    if text == "" and not required:
        return None
    if not _INT.fullmatch(text):
        raise PacketError("bad_field", f"{name} is not an integer: {text!r}")
    value = int(text)
    if not low <= value <= high:
        raise PacketError("bad_field", f"{name} out of range: {value}")
    return value


def _scaled(value: Optional[int], scale: float) -> Optional[float]:
    return None if value is None else value / scale


def decode(text: str) -> Packet:
    """Decodes one packet (trailing CR/LF allowed). Raises PacketError."""
    text = text.rstrip("\r\n")
    if len(text) > MAX_LEN:
        raise PacketError("too_long", f"{len(text)} chars, max {MAX_LEN}")

    match = _FRAME.fullmatch(text)
    if not match:
        raise PacketError("framing", f"not $...*XXXX: {text!r}")
    body, crc_hex = match.groups()
    try:
        body_bytes = body.encode("ascii")
    except UnicodeEncodeError:
        raise PacketError("framing", "non-ASCII characters") from None
    if crc16(body_bytes) != int(crc_hex, 16):
        raise PacketError("checksum", f"expected {crc16(body_bytes):04X}, got {crc_hex}")

    fields = body.split(",")
    if len(fields) != 9:
        raise PacketError("field_count", f"{len(fields)} fields, expected 9")
    tag, version, seq, time_ms, state, alt, pitch, yaw, batt = fields
    if tag != "TVR":
        raise PacketError("framing", f"unknown tag {tag!r}")
    if version != str(VERSION):
        raise PacketError("version", f"version {version!r}, expected {VERSION}")

    return Packet(
        seq=_int_field(seq, "seq", 0, 65535, required=True),
        time_ms=_int_field(time_ms, "time_ms", 0, 2**32 - 1, required=True),
        state=FlightState(_int_field(state, "state", 0, len(FlightState) - 1, required=True)),
        altitude_m=_scaled(_int_field(alt, "altitude", *ALT_DM_RANGE, required=False), 10),
        pitch_deg=_scaled(_int_field(pitch, "pitch", *ANGLE_DD_RANGE, required=False), 10),
        yaw_deg=_scaled(_int_field(yaw, "yaw", *ANGLE_DD_RANGE, required=False), 10),
        battery_v=_scaled(_int_field(batt, "battery", *BATT_MV_RANGE, required=False), 1000),
    )
