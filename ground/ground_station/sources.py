"""Where lines come from: a recorded raw log (--replay) or the dongle (--port).

Both yield (rx_time_s, line) so the rest of the station doesn't care which
one it's reading.
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Callable, Iterator, Optional, Tuple

from .logs import read_raw_log

# TODO(placeholder): confirm the dongle's baud rate with Yusuf.
DEFAULT_BAUD = 115200


def replay_lines(path: Path, speed: Optional[float] = 1.0,
                 sleep: Callable[[float], None] = time.sleep) -> Iterator[Tuple[float, str]]:
    """Plays back a raw log with its original timing, sped up by `speed`
    (2.0 = twice as fast). speed=None plays it back with no waiting.
    The recorded receive times are kept, so the output CSV matches the
    original session."""
    with open(path, newline="") as f:
        previous = None
        for rx_time_s, line in read_raw_log(f):
            if speed is not None and previous is not None:
                sleep(max(0.0, (rx_time_s - previous) / speed))
            previous = rx_time_s
            yield rx_time_s, line


def serial_lines(port: str, baud: int = DEFAULT_BAUD) -> Iterator[Tuple[float, str]]:
    """Reads lines from the dongle until interrupted, stamping each with the
    laptop's clock as it arrives."""
    import serial  # imported here so replay and tests don't need pyserial

    with serial.Serial(port, baud, timeout=1.0) as ser:
        while True:
            raw = ser.readline()
            if raw:
                yield time.time(), raw.decode("ascii", errors="replace")
