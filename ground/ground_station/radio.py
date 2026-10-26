"""Parses the lines the LoRa dongle prints over serial.

ASSUMPTION until Yusuf confirms the module: REYAX-style receive lines,

    +RCV=<address>,<length>,<data>,<rssi>,<snr>

where <data> is our packet. <data> itself contains commas, so it is cut
out by <length> instead of by splitting on commas. A line that is just a
packet ("$TVR,...") is also accepted, with no RSSI, so bench captures from
a plain serial link work too. If the real module differs, only this file
should need to change.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Optional

from .packet import PacketError

_NUMBER = re.compile(r"-?[0-9]+(\.[0-9]+)?")


@dataclass(frozen=True)
class RxFrame:
    payload: str               # the packet text, ready for packet.decode()
    rssi_dbm: Optional[int]    # None when the line had no radio info
    snr_db: Optional[float]


def parse_rx_line(line: str) -> Optional[RxFrame]:
    """Returns the frame in a received line, or None for lines that aren't
    data (module replies like +OK, +READY, blank lines). Raises PacketError
    with reason "rx_format" for a +RCV line that doesn't parse."""
    line = line.strip()
    if line.startswith("$"):
        return RxFrame(line, None, None)
    if not line.startswith("+RCV="):
        return None

    parts = line[len("+RCV="):].split(",", 2)
    if len(parts) != 3 or not parts[1].isdigit():
        raise PacketError("rx_format", f"bad +RCV header: {line!r}")
    length = int(parts[1])
    data, tail = parts[2][:length], parts[2][length:]

    radio = tail.split(",")
    if len(data) != length or len(radio) != 3 or radio[0] != "" \
            or not _NUMBER.fullmatch(radio[1]) or not _NUMBER.fullmatch(radio[2]):
        raise PacketError("rx_format", f"bad +RCV line: {line!r}")
    return RxFrame(data, int(float(radio[1])), float(radio[2]))
