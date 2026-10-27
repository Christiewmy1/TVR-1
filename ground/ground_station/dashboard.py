"""Live dashboard: altitude vs time, flight state, and signal strength.

Layout: altitude chart (top left), RSSI chart (bottom left, same time
axis), and a status panel (right) with the flight state as a big readout
plus the numbers an operator watches: altitude, max altitude, battery,
time since the last packet, and received/dropped/missed counts.

- DashboardModel holds the data and builds the panel text. No matplotlib,
  so it's unit-tested directly.
- DashboardView draws a model with matplotlib.
- run_dashboard() reads lines on a background thread (serial reads and
  replay waits block) and processes + redraws them on the main thread
  about 5 times a second, so all parsing and file writing stay on one
  thread.

Colours follow the data-viz reference palette: one series colour, hairline
recessive chrome, and status colours only for the link warning, always
with a text label.
"""

from __future__ import annotations

import queue
import threading
import time
from typing import Callable, Iterable, List, Optional, Tuple

from .station import Reading, Station

# Link warning thresholds: seconds since the last valid packet. At 5 Hz,
# 1 s is about 5 packets in a row lost.
NO_DATA_WARN_S = 1.0
NO_DATA_CRITICAL_S = 5.0
REFRESH_MS = 200

# Reference palette (light): surface, ink, chrome, series slot 1, status.
SURFACE = "#fcfcfb"
INK_PRIMARY = "#0b0b0b"
INK_SECONDARY = "#52514e"
INK_MUTED = "#898781"
GRIDLINE = "#e1e0d9"
BASELINE = "#c3c2b7"
SERIES = "#2a78d6"
STATUS_COLOURS = {"ok": "#0ca30c", "warning": "#fab219", "critical": "#d03b3b", "waiting": INK_MUTED}


def _fmt(value: Optional[float], unit: str, digits: int) -> str:
    return "--" if value is None else f"{value:.{digits}f} {unit}"


class DashboardModel:
    def __init__(self) -> None:
        self.t_s: List[float] = []            # rocket time of each altitude point
        self.alt_m: List[float] = []
        self.rssi_t_s: List[float] = []
        self.rssi_dbm: List[float] = []
        self.state_changes: List[Tuple[float, str]] = []  # (rocket time, new state)
        self.latest: Optional[Reading] = None

    def add(self, r: Reading) -> None:
        p = r.packet
        t = p.time_ms / 1000.0
        if p.altitude_m is not None:
            self.t_s.append(t)
            self.alt_m.append(p.altitude_m)
        if r.rssi_dbm is not None:
            self.rssi_t_s.append(t)
            self.rssi_dbm.append(float(r.rssi_dbm))
        if self.latest is not None and p.state != self.latest.packet.state:
            self.state_changes.append((t, p.state.name))
        self.latest = r

    @property
    def state(self) -> str:
        return "WAITING" if self.latest is None else self.latest.packet.state.name

    @property
    def max_altitude_m(self) -> Optional[float]:
        return None if self.latest is None else self.latest.packet.max_altitude_m

    def packet_age_s(self, now_s: float) -> Optional[float]:
        return None if self.latest is None else max(0.0, now_s - self.latest.rx_time_s)

    def link_status(self, now_s: float) -> Tuple[str, str]:
        """(status, label): status is ok / warning / critical / waiting."""
        age = self.packet_age_s(now_s)
        if age is None:
            return "waiting", "Waiting for first packet"
        if age >= NO_DATA_CRITICAL_S:
            return "critical", f"No data for {age:.0f} s"
        if age >= NO_DATA_WARN_S:
            return "warning", f"No data for {age:.1f} s"
        return "ok", "Receiving"

    def status_rows(self, station: Station, now_s: float) -> List[Tuple[str, str]]:
        p = self.latest.packet if self.latest else None
        age = self.packet_age_s(now_s)
        rssi = self.latest.rssi_dbm if self.latest else None
        return [
            ("Altitude", _fmt(p.altitude_m if p else None, "m", 1)),
            ("Max altitude", _fmt(self.max_altitude_m, "m", 1)),
            ("Battery", _fmt(p.battery_v if p else None, "V", 2)),
            ("Last packet", "--" if age is None else f"{age:.1f} s ago"),
            ("RSSI", "--" if rssi is None else f"{rssi} dBm"),
            ("Received", str(station.received)),
            ("Dropped", str(sum(station.dropped.values()))),
            ("Missed", str(station.missed)),
        ]


class DashboardView:
    def __init__(self, model: DashboardModel, title: str) -> None:
        import matplotlib.pyplot as plt  # here so the model works without matplotlib

        self.model = model
        self.title = title
        self.fig = plt.figure(figsize=(13, 7.5), facecolor=SURFACE)
        grid = self.fig.add_gridspec(3, 3, width_ratios=[1, 1, 0.75], hspace=0.45, wspace=0.3)
        self.ax_alt = self.fig.add_subplot(grid[0:2, 0:2])
        self.ax_rssi = self.fig.add_subplot(grid[2, 0:2], sharex=self.ax_alt)
        self.ax_status = self.fig.add_subplot(grid[:, 2])
        self.ax_status.axis("off")

        for ax, heading in [(self.ax_alt, "Altitude (m)"), (self.ax_rssi, "Signal strength, RSSI (dBm)")]:
            ax.set_facecolor(SURFACE)
            ax.set_title(heading, loc="left", color=INK_PRIMARY, fontsize=11)
            ax.grid(True, color=GRIDLINE, linewidth=0.8, linestyle="-")
            ax.set_axisbelow(True)
            ax.tick_params(colors=INK_MUTED, labelcolor=INK_SECONDARY, labelsize=9)
            for side in ("top", "right"):
                ax.spines[side].set_visible(False)
            for side in ("left", "bottom"):
                ax.spines[side].set_color(BASELINE)
        self.ax_rssi.set_xlabel("Flight computer time (s)", color=INK_SECONDARY, fontsize=9)

        (self.alt_line,) = self.ax_alt.plot([], [], color=SERIES, linewidth=2)
        (self.rssi_line,) = self.ax_rssi.plot([], [], color=SERIES, linewidth=2)
        self.max_line = self.ax_alt.axhline(0, color=INK_MUTED, linewidth=1, visible=False)
        # Max label sits at the left, where the curve is still near the ground.
        self.max_label = self.ax_alt.text(0.01, 0, "", transform=self.ax_alt.get_yaxis_transform(),
                                          ha="left", va="bottom", color=INK_SECONDARY, fontsize=9)
        self._markers: list = []
        self.fig.suptitle(title, x=0.02, ha="left", color=INK_PRIMARY, fontsize=13)

    def refresh(self, station: Station, now_s: float, banner: str = "") -> None:
        m = self.model
        self.alt_line.set_data(m.t_s, m.alt_m)
        self.rssi_line.set_data(m.rssi_t_s, m.rssi_dbm)
        for ax in (self.ax_alt, self.ax_rssi):
            ax.relim()
            ax.autoscale_view()
        if m.t_s:
            left, right = min(0.0, m.t_s[0]), max(m.t_s[-1], 1.0) * 1.02
            self.ax_alt.set_xlim(left, right)
            # Headroom above the peak keeps the state labels clear of it.
            low = min(0.0, min(m.alt_m))
            high = max(m.alt_m + ([m.max_altitude_m] if m.max_altitude_m is not None else []))
            span = max(high - low, 1.0)
            self.ax_alt.set_ylim(low - 0.05 * span, high + 0.3 * span)

        if m.max_altitude_m is not None:
            self.max_line.set_ydata([m.max_altitude_m, m.max_altitude_m])
            self.max_line.set_visible(True)
            self.max_label.set_position((0.01, m.max_altitude_m))
            self.max_label.set_text(f"max {m.max_altitude_m:.1f} m")

        self._draw_state_markers()

        self._draw_status(station, now_s)
        self.fig.suptitle(f"{self.title}{'   ·   ' + banner if banner else ''}",
                          x=0.02, ha="left", color=INK_PRIMARY, fontsize=13)

    def _draw_state_markers(self) -> None:
        """One hairline + label per state change, redrawn each refresh so the
        labels follow the growing time axis. Labels alternate rows so close
        changes (APOGEE then DESCENT) don't overlap, and flip to the left of
        their line near the right edge so they stay inside the chart."""
        for artist in self._markers:
            artist.remove()
        self._markers = []
        left, right = self.ax_alt.get_xlim()
        for i, (t, name) in enumerate(self.model.state_changes):
            near_right = t > right - 0.12 * (right - left)
            self._markers.append(self.ax_alt.axvline(t, color=BASELINE, linewidth=1))
            self._markers.append(self.ax_alt.text(
                t, 0.98 - 0.06 * (i % 2), f"{name} " if near_right else f" {name}",
                transform=self.ax_alt.get_xaxis_transform(), ha="right" if near_right else "left",
                va="top", color=INK_SECONDARY, fontsize=8))

    def _draw_status(self, station: Station, now_s: float) -> None:
        ax = self.ax_status
        ax.clear()
        ax.axis("off")
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)
        ax.text(0, 0.97, "Flight state", color=INK_SECONDARY, fontsize=10, va="top")
        ax.text(0, 0.92, self.model.state, color=INK_PRIMARY, fontsize=30, fontweight="bold", va="top")

        status, label = self.model.link_status(now_s)
        ax.scatter([0.03], [0.77], s=120, color=STATUS_COLOURS[status])
        ax.text(0.09, 0.77, label, color=INK_PRIMARY, fontsize=11, va="center")

        y = 0.67
        for name, value in self.model.status_rows(station, now_s):
            ax.text(0, y, name, color=INK_SECONDARY, fontsize=11, va="center")
            ax.text(1, y, value, color=INK_PRIMARY, fontsize=12, va="center", ha="right")
            y -= 0.075


_END = object()
_NON_INTERACTIVE = {"agg", "cairo", "pdf", "pgf", "ps", "svg", "template"}


def can_show_window() -> bool:
    """False when matplotlib has no screen to draw on (e.g. over SSH, or
    MPLBACKEND=Agg), in which case show() would return straight away."""
    import matplotlib.pyplot as plt

    return plt.get_backend().lower() not in _NON_INTERACTIVE


def _read_lines(lines: Iterable[Tuple[float, str]], q: "queue.Queue") -> None:
    try:
        for item in lines:
            q.put(item)
    except Exception as err:  # e.g. serial port missing: report it on the main thread
        q.put(err)
    finally:
        q.put(_END)


def run_dashboard(lines: Iterable[Tuple[float, str]],
                  handle: Callable[[float, str], Optional[Reading]],
                  station: Station, title: str, live: bool) -> None:
    """Shows the dashboard until the window is closed. `handle` processes one
    line (decode, log, print) and returns the Reading for a valid packet."""
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    model = DashboardModel()
    view = DashboardView(model, title)
    q: "queue.Queue" = queue.Queue()
    threading.Thread(target=_read_lines, args=(lines, q), daemon=True).start()
    state = {"last_rx": None, "banner": ""}

    def tick(_frame: int) -> None:
        while True:
            try:
                item = q.get_nowait()
            except queue.Empty:
                break
            if item is _END:
                if not state["banner"]:
                    state["banner"] = "connection ended" if live else "replay finished"
            elif isinstance(item, Exception):
                state["banner"] = f"error: {item}"
                print(f"error reading input: {item}")
            else:
                rx_time_s, line = item
                state["last_rx"] = rx_time_s
                reading = handle(rx_time_s, line)
                if reading:
                    model.add(reading)
        now = time.time() if live else (state["last_rx"] or 0.0)
        view.refresh(station, now, state["banner"])

    animation = FuncAnimation(view.fig, tick, interval=REFRESH_MS, cache_frame_data=False)
    plt.show()
    del animation
