"""Command line: python -m ground_station (--replay LOG | --port DEVICE)

Reads lines, prints each valid packet, writes the telemetry CSV (and, when
reading the real dongle, a raw log for replay later), and prints the
received / dropped / missed counts at the end. Ctrl+C stops it cleanly.
"""

from __future__ import annotations

import argparse
import time
from contextlib import ExitStack
from pathlib import Path
from typing import List, Optional

from .logs import RawLog, TelemetryCsv
from .sources import DEFAULT_BAUD, replay_lines, serial_lines
from .station import Reading, Station


def _fmt(value: Optional[float], unit: str, digits: int) -> str:
    return "  --" if value is None else f"{value:.{digits}f}{unit}"


def describe(r: Reading) -> str:
    p = r.packet
    rssi = "" if r.rssi_dbm is None else f"  rssi {r.rssi_dbm} dBm"
    return (f"#{p.seq:<5} t={p.time_ms / 1000:8.3f}s  {p.state.name:<7}  "
            f"alt {_fmt(p.altitude_m, ' m', 1)} (max {_fmt(p.max_altitude_m, ' m', 1)})  batt {_fmt(p.battery_v, ' V', 2)}{rssi}")


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="ground_station",
        description="TVR-1 ground station: read telemetry from the dongle or a recorded raw log.")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--replay", type=Path, metavar="LOG", help="raw log to play back")
    source.add_argument("--port", metavar="DEVICE", help="dongle serial port, e.g. /dev/tty.usbserial-XXXX")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"serial baud rate (default {DEFAULT_BAUD})")
    parser.add_argument("--speed", type=float, default=1.0, help="replay speed factor (default 1.0)")
    parser.add_argument("--fast", action="store_true", help="replay with no waiting")
    parser.add_argument("--out", type=Path, default=Path("logs"), help="folder for output files (default ./logs)")
    parser.add_argument("--quiet", action="store_true", help="don't print each packet")
    args = parser.parse_args(argv)

    args.out.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    station = Station()

    with ExitStack() as files:
        telemetry = TelemetryCsv(files.enter_context(open(args.out / f"telemetry_{stamp}.csv", "w", newline="")))
        raw = None
        if args.port:
            raw = RawLog(files.enter_context(open(args.out / f"raw_{stamp}.csv", "w", newline="")))
            lines = serial_lines(args.port, args.baud)
        else:
            lines = replay_lines(args.replay, speed=None if args.fast else args.speed)

        print(f"writing to {args.out}/ ({stamp}); Ctrl+C to stop")
        try:
            for rx_time_s, line in lines:
                if raw:
                    raw.write(rx_time_s, line)
                reading = station.handle(line, rx_time_s)
                if reading:
                    telemetry.write(reading)
                    if not args.quiet:
                        print(describe(reading))
        except KeyboardInterrupt:
            pass

    print(station.summary())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
