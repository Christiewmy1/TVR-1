"""The two files the ground station writes.

- Telemetry CSV: one row per valid packet, in engineering units, with the
  ground receive time. This is the flight data to analyse afterwards.
- Raw log: every line the dongle printed, with its receive time. It's what
  --replay reads, so a recorded session can be re-run through the real
  parsing, bad packets included.

Both are plain CSV written with the csv module (raw lines contain commas,
so they get quoted).
"""

from __future__ import annotations

import csv
from typing import IO, Iterator, Optional, Tuple

from .station import Reading

TELEMETRY_FIELDS = ["rx_time_s", "seq", "time_ms", "state", "altitude_m", "pitch_deg",
                    "yaw_deg", "battery_v", "rssi_dbm", "snr_db"]
RAW_FIELDS = ["rx_time_s", "line"]


def _cell(value: Optional[object]) -> object:
    return "" if value is None else value


class TelemetryCsv:
    def __init__(self, f: IO[str]) -> None:
        self._f = f
        self._writer = csv.writer(f)
        self._writer.writerow(TELEMETRY_FIELDS)

    def write(self, r: Reading) -> None:
        p = r.packet
        self._writer.writerow([
            f"{r.rx_time_s:.3f}", p.seq, p.time_ms, p.state.name,
            _cell(p.altitude_m), _cell(p.pitch_deg), _cell(p.yaw_deg), _cell(p.battery_v),
            _cell(r.rssi_dbm), _cell(r.snr_db),
        ])
        self._f.flush()  # keep the file current if the laptop dies mid-flight


class RawLog:
    def __init__(self, f: IO[str]) -> None:
        self._f = f
        self._writer = csv.writer(f)
        self._writer.writerow(RAW_FIELDS)

    def write(self, rx_time_s: float, line: str) -> None:
        self._writer.writerow([f"{rx_time_s:.3f}", line.rstrip("\r\n")])
        self._f.flush()


def read_raw_log(f: IO[str]) -> Iterator[Tuple[float, str]]:
    """Yields (rx_time_s, line) from a raw log written by RawLog."""
    reader = csv.reader(f)
    header = next(reader, None)
    if header != RAW_FIELDS:
        raise ValueError(f"not a raw log (header {header!r}, expected {RAW_FIELDS})")
    for row in reader:
        yield float(row[0]), row[1]
