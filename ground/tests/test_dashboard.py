"""Tests for the live dashboard (ground_station/dashboard.py).

The model is tested directly. The view is drawn with matplotlib's off-screen
Agg backend (no window), fed the whole sample flight, so CI can run it.
"""

import queue
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

from ground_station.dashboard import (  # noqa: E402  (backend must be set first)
    NO_DATA_CRITICAL_S, NO_DATA_WARN_S, _END, DashboardModel, DashboardView, _read_lines)
from ground_station.packet import FlightState, Packet  # noqa: E402
from ground_station.sources import replay_lines  # noqa: E402
from ground_station.station import Reading, Station  # noqa: E402

SAMPLE = Path(__file__).parent / "data" / "sample_flight_raw.csv"


def reading(time_ms, state=FlightState.BOOST, alt=10.0, max_alt=10.0, rssi=-60, rx=100.0):
    return Reading(rx, Packet(0, time_ms, state, alt, max_alt, 0.0, 0.0, 12.3), rssi, 9.0)


def test_empty_model_is_waiting():
    m = DashboardModel()
    assert m.state == "WAITING"
    assert m.link_status(now_s=0.0) == ("waiting", "Waiting for first packet")
    assert dict(m.status_rows(Station(), 0.0))["Altitude"] == "--"


def test_series_skip_missing_readings():
    m = DashboardModel()
    m.add(reading(1000, alt=5.0))
    m.add(reading(1200, alt=None, rssi=None))
    m.add(reading(1400, alt=7.5, rssi=-70))
    assert m.t_s == [1.0, 1.4]
    assert m.alt_m == [5.0, 7.5]
    assert m.rssi_t_s == [1.0, 1.4]
    assert m.rssi_dbm == [-60.0, -70.0]


def test_state_changes_recorded_only_on_change():
    m = DashboardModel()
    for t, state in [(0, FlightState.PAD), (200, FlightState.PAD), (400, FlightState.BOOST),
                     (600, FlightState.BOOST), (800, FlightState.COAST)]:
        m.add(reading(t, state=state))
    assert m.state_changes == [(0.4, "BOOST"), (0.8, "COAST")]
    assert m.state == "COAST"


def test_link_status_thresholds():
    m = DashboardModel()
    m.add(reading(0, rx=100.0))
    assert m.link_status(100.5)[0] == "ok"
    assert m.link_status(100.0 + NO_DATA_WARN_S) == ("warning", "No data for 1.0 s")
    assert m.link_status(100.0 + NO_DATA_CRITICAL_S)[0] == "critical"


def test_status_rows():
    m = DashboardModel()
    m.add(reading(0, alt=123.45, max_alt=150.0, rssi=-72, rx=100.0))
    station = Station()
    station.received, station.missed = 10, 2
    station.dropped["checksum"] = 3
    rows = dict(m.status_rows(station, now_s=100.25))
    assert rows == {
        "Altitude": "123.5 m", "Max altitude": "150.0 m", "Battery": "12.30 V",
        "Last packet": "0.2 s ago", "RSSI": "-72 dBm",
        "Received": "10", "Dropped": "3", "Missed": "2",
    }


def test_reader_thread_passes_lines_then_end_and_reports_errors():
    def lines():
        yield 1.0, "+OK"
        raise OSError("port gone")

    q = queue.Queue()
    _read_lines(lines(), q)
    items = [q.get_nowait() for _ in range(q.qsize())]
    assert items[0] == (1.0, "+OK")
    assert isinstance(items[1], OSError)
    assert items[2] is _END


def test_view_draws_the_whole_sample_flight(tmp_path):
    model, station = DashboardModel(), Station()
    last_rx = 0.0
    for rx, line in replay_lines(SAMPLE, speed=None):
        r = station.handle(line, rx)
        last_rx = rx
        if r:
            model.add(r)

    view = DashboardView(model, "TVR-1 ground station   ·   replay: sample_flight_raw.csv")
    view.refresh(station, now_s=last_rx, banner="replay finished")

    assert len(view.alt_line.get_xdata()) == len(model.t_s) == station.received
    assert [name for _, name in model.state_changes] == ["BOOST", "COAST", "APOGEE", "DESCENT", "LANDED"]
    assert view.max_label.get_text() == "max 265.7 m"
    view.fig.savefig(tmp_path / "dashboard.png")  # renders without error


def test_without_a_display_the_command_falls_back_to_printing(tmp_path, capsys):
    # This test module runs on the Agg backend, i.e. no screen.
    from ground_station.__main__ import main

    assert main(["--replay", str(SAMPLE), "--fast", "--quiet", "--out", str(tmp_path)]) == 0
    out = capsys.readouterr().out
    assert "no display available" in out
    assert "received 172" in out
