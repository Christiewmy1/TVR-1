"""Tests for the station's parse -> validate -> count logic (ground_station/station.py)."""

from ground_station.packet import FlightState, crc16
from ground_station.station import Station


def rcv(seq, time_ms, state=0, alt="100", rssi=-50):
    """A +RCV line carrying a valid packet."""
    body = f"TVR,1,{seq},{time_ms},{state},{alt},{alt},0,0,7400"
    packet = f"${body}*{crc16(body.encode('ascii')):04X}"
    return f"+RCV=1,{len(packet)},{packet},{rssi},9.0"


def test_valid_line_gives_a_reading():
    s = Station()
    r = s.handle(rcv(5, 1000, state=1, alt="515"), rx_time_s=123.5)
    assert r.rx_time_s == 123.5
    assert r.packet.seq == 5
    assert r.packet.state == FlightState.BOOST
    assert r.packet.altitude_m == 51.5
    assert r.rssi_dbm == -50
    assert s.received == 1
    assert not s.dropped


def test_bad_packets_are_counted_by_reason():
    s = Station()
    good = rcv(0, 0)
    corrupted = good.replace(",100,", ",101,")
    assert s.handle(corrupted, 0.0) is None
    assert s.handle("+RCV=1,28,$TVR", 0.0) is None
    assert s.handle("$garbage", 0.0) is None
    assert s.dropped == {"checksum": 1, "rx_format": 1, "framing": 1}
    assert s.received == 0


def test_module_replies_are_ignored_not_counted():
    s = Station()
    assert s.handle("+OK", 0.0) is None
    assert s.received == 0 and not s.dropped


def test_seq_gaps_count_as_missed():
    s = Station()
    for seq in [0, 1, 4, 5]:
        s.handle(rcv(seq, seq * 100), 0.0)
    assert s.missed == 2


def test_seq_wraparound_is_not_a_gap():
    s = Station()
    s.handle(rcv(65534, 1000), 0.0)
    s.handle(rcv(65535, 1100), 0.0)
    s.handle(rcv(0, 1200), 0.0)
    assert s.missed == 0
    s.handle(rcv(3, 1500), 0.0)
    assert s.missed == 2


def test_reboot_restarting_seq_is_not_counted_as_missed():
    s = Station()
    s.handle(rcv(500, 50000), 0.0)
    s.handle(rcv(0, 20), 0.0)  # rocket rebooted: clock and seq restarted
    assert s.missed == 0


def test_summary():
    s = Station()
    s.handle(rcv(0, 0), 0.0)
    s.handle(rcv(2, 200), 0.0)
    s.handle("$garbage", 0.0)
    assert s.summary() == "received 2, dropped 1 (framing 1), missed 1 (seq gaps: lost on air or dropped)"
