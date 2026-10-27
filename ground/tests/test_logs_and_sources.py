"""Tests for the CSV/raw log files (logs.py) and replay timing (sources.py)."""

import csv
import io

import pytest

from ground_station.logs import RawLog, TelemetryCsv, read_raw_log
from ground_station.packet import FlightState, Packet
from ground_station.sources import replay_lines
from ground_station.station import Reading


def test_telemetry_csv_row_with_missing_readings():
    out = io.StringIO()
    TelemetryCsv(out).write(Reading(
        rx_time_s=1793000000.0124,
        packet=Packet(7, 702, FlightState.COAST, 128.0, 130.5, None, None, 7.39),
        rssi_dbm=None, snr_db=None,
    ))
    rows = list(csv.reader(io.StringIO(out.getvalue())))
    assert rows[0][:4] == ["rx_time_s", "seq", "time_ms", "state"]
    assert rows[1] == ["1793000000.012", "7", "702", "COAST", "128.0", "130.5", "", "", "7.39", "", ""]


def test_raw_log_round_trip_keeps_lines_with_commas():
    out = io.StringIO()
    log = RawLog(out)
    log.write(10.0, "+RCV=1,29,$TVR,1,0,0,0,0,,0,0,7400*1260,-45,9.5\r\n")
    log.write(10.5, "+OK")
    lines = list(read_raw_log(io.StringIO(out.getvalue())))
    assert lines == [(10.0, "+RCV=1,29,$TVR,1,0,0,0,0,,0,0,7400*1260,-45,9.5"), (10.5, "+OK")]


def test_read_raw_log_rejects_other_csv_files():
    with pytest.raises(ValueError):
        list(read_raw_log(io.StringIO("rx_time_s,seq,time_ms\n1,2,3\n")))


def make_log(tmp_path):
    path = tmp_path / "raw.csv"
    with open(path, "w", newline="") as f:
        log = RawLog(f)
        for t in [100.0, 100.5, 101.5]:
            log.write(t, "+OK")
    return path


@pytest.mark.parametrize("speed, expected_sleeps", [(1.0, [0.5, 1.0]), (2.0, [0.25, 0.5]), (None, [])])
def test_replay_timing(tmp_path, speed, expected_sleeps):
    sleeps = []
    lines = list(replay_lines(make_log(tmp_path), speed=speed, sleep=sleeps.append))
    assert [t for t, _ in lines] == [100.0, 100.5, 101.5]
    assert sleeps == pytest.approx(expected_sleeps)
