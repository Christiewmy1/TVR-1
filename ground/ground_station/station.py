"""The ground station's processing: parse -> validate -> count.

Station.handle() takes one line from the dongle (live or replayed) and
returns a Reading for a valid packet, or None. It keeps the counts the
operator needs: packets received, packets dropped (by reason), and packets
missed (gaps in the sequence number). A gap can't tell a packet lost on air
from one that arrived corrupted and was dropped, so `missed` counts both.
No I/O here, so it's easy to test.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
from typing import Optional

from .packet import Packet, PacketError, decode
from .radio import parse_rx_line

SEQ_MODULO = 65536  # seq wraps 65535 -> 0


@dataclass(frozen=True)
class Reading:
    """A valid packet as received on the ground."""

    rx_time_s: float          # ground clock, Unix seconds
    packet: Packet
    rssi_dbm: Optional[int]
    snr_db: Optional[float]


class Station:
    def __init__(self) -> None:
        self.received = 0
        self.dropped: Counter = Counter()  # reason -> count
        self.missed = 0                    # seq numbers never received intact
        self._last: Optional[Packet] = None

    def handle(self, line: str, rx_time_s: float) -> Optional[Reading]:
        try:
            frame = parse_rx_line(line)
            if frame is None:
                return None
            packet = decode(frame.payload)
        except PacketError as err:
            self.dropped[err.reason] += 1
            return None

        self._count_missed(packet)
        self._last = packet
        self.received += 1
        return Reading(rx_time_s, packet, frame.rssi_dbm, frame.snr_db)

    def _count_missed(self, packet: Packet) -> None:
        last = self._last
        # If the rocket's clock went backwards it rebooted and seq restarted:
        # don't count that jump as missed packets.
        if last is None or packet.time_ms < last.time_ms:
            return
        self.missed += (packet.seq - last.seq - 1) % SEQ_MODULO

    def summary(self) -> str:
        dropped = sum(self.dropped.values())
        reasons = ", ".join(f"{r} {n}" for r, n in sorted(self.dropped.items()))
        return (f"received {self.received}, dropped {dropped}"
                f"{f' ({reasons})' if reasons else ''}, "
                f"missed {self.missed} (seq gaps: lost on air or dropped)")
