# TVR-1

Student rocket with thrust vector control: the motor mount gimbals on two servo axes to steer.

## Repo layout

| Folder | What's in it | Owner |
|---|---|---|
| `firmware/` | Flight software in C (state machine, PID, telemetry encoder). Runs on the STM32F405 eventually; built and tested on a laptop for now. | Software |
| `ground/` | Ground station in Python: reads telemetry from the LoRa dongle, logs to CSV, live dashboard. | Software |
| `tests/` | C unit tests (Unity) and shared test data. | Software |
| `Simulation/` | MATLAB PID sims, OpenRocket file, sim exports. | Team lead |
| `hardware/` | Flight controller + USB dongle (KiCad, gerbers, BOM). | Hardware |
| `mechanical/` | Gimbal, e-bay, dongle enclosure, recovery CAD. | Mechanical |
| `docs/` | Design docs by subteam. | All |

## Software: building and testing

Requirements: a C compiler (`clang` on macOS, `gcc` on Linux), `make`, and Python 3.9+.

**C unit tests**

```bash
make test     # build and run all C tests into build/
make clean
```

All firmware C is standard C99, compiled with `-std=c99 -Wall -Wextra -Werror`, with no `malloc`, no OS calls and no hardware code, so it builds unchanged on a laptop and on the STM32. Hardware drivers live in a separate layer that calls into these modules.

**Python tests** (first time: create a virtual environment)

```bash
cd ground
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python -m pytest
```

**Ground station** (from `ground/`, with the virtual environment active)

```bash
python -m ground_station --replay tests/data/sample_flight_raw.csv          # play back a recorded log
python -m ground_station --replay tests/data/sample_flight_raw.csv --speed 4  # ...4x faster
python -m ground_station --port /dev/tty.usbserial-XXXX                    # the real dongle
python -m ground_station --replay tests/data/sample_flight_raw.csv --fast --no-plot  # no window, just print
```

It opens a live dashboard showing:
- altitude vs time, with the max altitude and each state change marked
- signal strength (RSSI)
- the current flight state, a warning when no data has arrived for 1 s or more, and the key numbers

It also prints each valid packet and writes `logs/telemetry_<time>.csv`. With `--port` it also writes `logs/raw_<time>.csv`, a copy of every line from the dongle that `--replay` can play back later. At the end it prints how many packets were received, dropped (by reason), and missed.

`make sample-log` regenerates `ground/tests/data/sample_flight_raw.csv` from an OpenRocket flight, using the real C state machine and telemetry encoder.

**CI:** GitHub Actions (`.github/workflows/ci.yml`) runs both test suites on every push and pull request.

## Placeholder values

Thresholds, gains and limits that the team hasn't confirmed yet are in `firmware/include/config.h`, marked `TODO(placeholder)`. Run `grep -rn "TODO(placeholder)" firmware/` to list them. None may remain before flight.

## Contributing

- One branch per feature (`feature/state-machine`, `feature/packet-spec`, ...). Don't commit straight to `main`.
- Open a pull request; CI must pass before merging.
- New functions get a unit test; bug fixes get a test that would have caught the bug.
