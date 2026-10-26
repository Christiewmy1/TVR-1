# TVR-1 Telemetry Packet — DRAFT v1

**Status: draft.** This isn't final until the LoRa module is confirmed and Yusuf (ground station) and Samantha (flight computer) sign off below.
Code: `firmware/src/telemetry.c` (encoder) and `ground/ground_station/packet.py` (decoder). Both are tested against the shared vectors in `tests/data/telemetry_vectors.csv`.

## Format

One packet is one line of printable ASCII text, at most **54 characters**, sent as the payload of the LoRa module's AT send command:

```
$TVR,<ver>,<seq>,<time_ms>,<state>,<alt>,<pitch>,<yaw>,<batt>*<CRC>
$TVR,1,42,1234,1,515,20,-15,7312*3576
```

| # | Field | Units / encoding | Range | Example |
|---|---|---|---|---|
| 1 | Tag | always `TVR` | | `TVR` |
| 2 | Version | format version | `1` | `1` |
| 3 | `seq` | packet counter, wraps 65535 → 0 | 0 – 65535 | `42` |
| 4 | `time_ms` | ms since flight computer boot | 0 – 4294967295 | `1234` |
| 5 | `state` | flight state code (table below) | 0 – 5 | `1` |
| 6 | `alt` | altitude above launch site, **0.1 m** | −9999 – 99999 (−999.9 – 9999.9 m) | `515` = 51.5 m |
| 7 | `pitch` | tilt from vertical, pitch axis, **0.1°** | −1800 – 1800 | `20` = 2.0° |
| 8 | `yaw` | tilt from vertical, yaw axis, **0.1°** | −1800 – 1800 | `-15` = −1.5° |
| 9 | `batt` | battery voltage, **mV** | 0 – 65535 | `7312` = 7.312 V |
| | `CRC` | CRC-16 as 4 hex digits (below) | | `3576` |

- **Numbers** are plain integers: optional `-`, digits only. No `+`, no decimal point, no spaces.
- **Out-of-range values** are clamped to the range limit by the encoder.
- **A missing reading** (sensor marked invalid) is sent as an **empty field**, e.g. `$TVR,1,3,100,2,,,,*2B46`. Fields 6–9 can be empty; 1–5 are always present.

**State codes** (same numbers as the `FlightState` enum): 0 PAD · 1 BOOST · 2 COAST · 3 APOGEE · 4 DESCENT · 5 LANDED.

**Checksum:** CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over every character between `$` and `*`, written as 4 hex digits (uppercase when sent; either case is accepted). Check value: `"123456789"` → `29B1`. It's a stronger check than an NMEA-style 8-bit XOR for 2 extra characters, and Python has it built in (`binascii.crc_hqx`).

## Size

The widest possible packet is 54 characters; a typical one is 30–43. **The radio's max payload is TBD.** `config.h` has a 240-byte placeholder until the module is confirmed, and a unit test checks that 54 fits in it. The send rate is also TBD.

## Ground station checks

Every rejected packet is counted by reason:

| Reason | Meaning |
|---|---|
| `too_long` | more than 54 characters |
| `framing` | not `$…*XXXX`, wrong tag, or non-ASCII |
| `checksum` | CRC doesn't match |
| `field_count` | not exactly 9 fields |
| `version` | version isn't 1 |
| `bad_field` | a field isn't an integer, is out of range, or a required field is empty |

**RSSI** isn't in the packet. The receiving module reports it with each received line.

## Why this design

- **Text, not binary.** The radio takes AT commands as text. Binary bytes could contain line breaks or commas that break the command, and text can be read directly in a serial terminal while debugging.
- **Scaled integers.** No float formatting on the flight computer (no `printf`). Every value has a fixed, known resolution.
- **Version field.** If the format changes after sign-off, an old ground station rejects new packets (`version`) instead of misreading them.

## Open questions

- **Yusuf:**
  - Which LoRa module, its max payload, and its baud rate?
  - Can the payload contain `,`, `$` and `*`?
  - What exactly is the receive line? If it's REYAX-style `+RCV=<addr>,<len>,<data>,<rssi>,<snr>`, the ground station will take `<len>` characters of data rather than split on commas.
- **Samantha:** Is 0–65.535 V right for the battery? What will provide the pitch/yaw tilt?
- **Team lead / Ethan:** Is pitch/yaw tilt the right "orientation" (is roll needed)? What send rate?

## Sign-off

| Name | Role | Date |
|---|---|---|
| Yusuf | Ground station hardware | |
| Samantha | Flight computer | |
