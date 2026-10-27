"""Tests for the telemetry decoder (ground_station/packet.py).

The golden vectors are shared with tests/test_telemetry.c, which checks the
C encoder produces exactly these packets. Here we check Python decodes them
back to the values the rocket started with (to within one step of the
field's resolution, after clamping).
"""

import math
from pathlib import Path

import pytest

from ground_station.packet import FlightState, PacketError, crc16, decode

PAD = "$TVR,1,0,0,0,0,,0,0,7400*1260"  # golden vector: on the pad
VECTORS = Path(__file__).resolve().parents[2] / "tests" / "data" / "telemetry_vectors.csv"


def frame(body: str) -> str:
    """Wraps a packet body with a valid checksum, to test other checks."""
    return f"${body}*{crc16(body.encode('ascii')):04X}"


def load_vectors():
    rows = []
    for line in VECTORS.read_text().splitlines():
        if line.startswith("#") or not line:
            continue
        inputs, packet = line.split("|")
        rows.append(([float(x) for x in inputs.split(",")], packet))
    return rows


def expected(value, low, high):
    return None if math.isnan(value) else min(max(value, low), high)


# ---- golden vectors ----------------------------------------------------------

def test_vectors_file_has_every_row():
    assert len(load_vectors()) == 8


@pytest.mark.parametrize("inputs, packet", load_vectors())
def test_decodes_golden_vector(inputs, packet):
    seq, time_ms, state, alt, max_alt, pitch, yaw, batt = inputs
    p = decode(packet)

    assert p.seq == seq
    assert p.time_ms == time_ms
    assert p.state == FlightState(int(state))
    for got, value, low, high, step in [
        (p.altitude_m, alt, -999.9, 9999.9, 0.1),
        (p.max_altitude_m, max_alt, -999.9, 9999.9, 0.1),
        (p.pitch_deg, pitch, -180.0, 180.0, 0.1),
        (p.yaw_deg, yaw, -180.0, 180.0, 0.1),
        (p.battery_v, batt, 0.0, 65.535, 0.001),
    ]:
        want = expected(value, low, high)
        if want is None:
            assert got is None
        else:
            assert got == pytest.approx(want, abs=step / 2 + 1e-9)


def test_crc16_standard_check_value():
    assert crc16(b"123456789") == 0x29B1


def test_trailing_newlines_are_ignored():
    assert decode(PAD + "\r\n").battery_v == 7.4


def test_no_max_altitude_before_launch():
    assert decode(PAD).max_altitude_m is None


def test_lowercase_checksum_hex_is_accepted():
    assert decode("$TVR,1,42,1234,1,515,515,20,-15,7312*d87c").seq == 42


# ---- rejects (each must give the right reason, for drop counting) -------------

@pytest.mark.parametrize("packet, reason", [
    (PAD.replace("*1260", "*1261"), "checksum"),                     # CRC off by one
    (PAD.replace("7400", "7401"), "checksum"),                       # corrupted field
    (PAD[1:], "framing"),                                            # no '$'
    (PAD[:-5], "framing"),                                           # no checksum
    (PAD[:-2], "framing"),                                           # short checksum
    (PAD.replace("7400", "74\u00b000"), "framing"),                   # non-ASCII
    (PAD + "0" * 40, "too_long"),
    (frame("TVR,1,0,0,0,0,0,0,7400"), "field_count"),               # field missing
    (frame("TVR,1,0,0,0,0,0,0,0,7400,9"), "field_count"),           # extra field
    (frame("ABC,1,0,0,0,0,0,0,0,7400"), "framing"),                  # wrong tag
    (frame("TVR,2,0,0,0,0,0,0,0,7400"), "version"),
    (frame("TVR,1,,0,0,0,0,0,0,7400"), "bad_field"),                 # seq is required
    (frame("TVR,1,0,0,6,0,0,0,0,7400"), "bad_field"),                # no state 6
    (frame("TVR,1,65536,0,0,0,0,0,0,7400"), "bad_field"),            # seq too big
    (frame("TVR,1,0,0,0,100000,0,0,0,7400"), "bad_field"),           # altitude out of range
    (frame("TVR,1,0,0,0,0,100000,0,0,7400"), "bad_field"),           # max altitude out of range
    (frame("TVR,1,0,0,0,0,0,1801,0,7400"), "bad_field"),             # angle out of range
    (frame("TVR,1,0,0,0,0,0,0,0,-1"), "bad_field"),                  # negative battery
    (frame("TVR,1,0,0,0,12a,0,0,0,7400"), "bad_field"),              # not a number
    (frame("TVR,1,0,0,0,+5,0,0,0,7400"), "bad_field"),               # '+' not allowed
    (frame("TVR,1,0,0,0,1.5,0,0,0,7400"), "bad_field"),              # not an integer
])
def test_rejects_with_reason(packet, reason):
    with pytest.raises(PacketError) as err:
        decode(packet)
    assert err.value.reason == reason
