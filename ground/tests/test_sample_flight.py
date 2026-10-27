"""End-to-end: replay the sample dongle log through the whole station.

tests/data/sample_flight_raw.csv was made by `make sample-log` from the
updated-airframe OpenRocket flight using the real C state machine and
telemetry encoder, so this checks C encoder -> Python decoder on a full
flight. The faults it contains are deliberate (see
tests/tools/make_sample_log.c); the counts below follow from its rules.
"""

import csv
from pathlib import Path

from ground_station.__main__ import main
from ground_station.sources import replay_lines
from ground_station.station import Station

SAMPLE = Path(__file__).parent / "data" / "sample_flight_raw.csv"


def replay():
    station, readings = Station(), []
    for rx_time_s, line in replay_lines(SAMPLE, speed=None):
        reading = station.handle(line, rx_time_s)
        if reading:
            readings.append(reading)
    return station, readings


def test_counts_match_the_injected_faults():
    station, _ = replay()
    # 179 packets sent: seq 13 and 110 lost; 7, 57, 107, 157 corrupted; 75 cut off.
    assert station.received == 172
    assert station.dropped == {"checksum": 4, "rx_format": 1}
    assert station.missed == 7  # 2 lost + 5 dropped


def test_states_arrive_in_flight_order():
    _, readings = replay()
    seen = []
    for r in readings:
        if not seen or seen[-1] != r.packet.state.name:
            seen.append(r.packet.state.name)
    # APOGEE lasts one sensor sample on the rocket; it only shows up because
    # every state change sends a packet straight away.
    assert seen == ["PAD", "BOOST", "COAST", "APOGEE", "DESCENT", "LANDED"]


def test_max_altitude_field_carries_the_peak_to_landing():
    _, readings = replay()
    assert readings[0].packet.max_altitude_m is None  # on the pad
    assert abs(readings[-1].packet.max_altitude_m - 265.7) <= 0.1  # OpenRocket apogee 265.728 m


def test_command_line_replay_writes_every_valid_packet(tmp_path, capsys):
    assert main(["--replay", str(SAMPLE), "--fast", "--quiet", "--no-plot", "--out", str(tmp_path)]) == 0
    (telemetry_csv,) = tmp_path.glob("telemetry_*.csv")
    with open(telemetry_csv, newline="") as f:
        assert sum(1 for _ in csv.reader(f)) == 1 + 172  # header + rows
    assert "received 172, dropped 5" in capsys.readouterr().out
