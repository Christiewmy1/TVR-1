"""Command line: python -m ground_station (--replay LOG | --port DEVICE)

Opens the live dashboard (or, with --no-plot, just prints), writes the
telemetry CSV (and, when reading the real dongle, a raw log for replay
later), and prints the received / dropped / missed counts at the end.
Close the window or press Ctrl+C to stop.
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


class Session:
    """What happens to every line, with or without the dashboard: save it to
    the raw log (live only), decode it, and write/print valid packets."""

    def __init__(self, telemetry: TelemetryCsv, raw: Optional[RawLog], quiet: bool) -> None:
        self.station = Station()
        self._telemetry = telemetry
        self._raw = raw
        self._quiet = quiet

    def handle(self, rx_time_s: float, line: str) -> Optional[Reading]:
        if self._raw:
            self._raw.write(rx_time_s, line)
        reading = self.station.handle(line, rx_time_s)
        if reading:
            self._telemetry.write(reading)
            if not self._quiet:
                print(describe(reading))
        return reading


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
    parser.add_argument("--no-plot", action="store_true", help="no dashboard window, just print")
    args = parser.parse_args(argv)

    args.out.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")

    with ExitStack() as files:
        telemetry = TelemetryCsv(files.enter_context(open(args.out / f"telemetry_{stamp}.csv", "w", newline="")))
        raw = None
        if args.port:
            raw = RawLog(files.enter_context(open(args.out / f"raw_{stamp}.csv", "w", newline="")))
            lines = serial_lines(args.port, args.baud)
            title = f"TVR-1 ground station   ·   live: {args.port}"
        else:
            lines = replay_lines(args.replay, speed=None if args.fast else args.speed)
            title = f"TVR-1 ground station   ·   replay: {args.replay.name}"
        session = Session(telemetry, raw, args.quiet)

        plot = not args.no_plot
        if plot:
            from .dashboard import can_show_window, run_dashboard
            if not can_show_window():
                print("no display available, running without the dashboard (use --no-plot to skip this check)")
                plot = False

        print(f"writing to {args.out}/ ({stamp}); {'close the window' if plot else 'Ctrl+C'} to stop")
        try:
            if plot:
                run_dashboard(lines, session.handle, session.station, title, live=bool(args.port))
            else:
                for rx_time_s, line in lines:
                    session.handle(rx_time_s, line)
        except KeyboardInterrupt:
            pass

    print(session.station.summary())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
