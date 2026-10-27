/*
 * telemetry.h - telemetry packet encoder and send schedule. The format is
 * specified in docs/packet_spec.md (DRAFT until Yusuf and Samantha sign
 * off).
 *
 * A packet is one line of plain ASCII text, e.g.
 *
 *   $TVR,1,42,1234,1,515,515,20,-15,7312*XXXX
 *
 * so it can go straight into the LoRa module's AT send command. Values are
 * sent as scaled integers (altitudes in 0.1 m, angles in 0.1 deg, battery
 * in mV), so no float formatting is needed on the flight computer.
 */
#ifndef TVR1_TELEMETRY_H
#define TVR1_TELEMETRY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "flight_state.h"

#define TELEMETRY_VERSION       1u

/* Longest possible packet, in characters (no terminator): every field at
 * its widest. See docs/packet_spec.md. */
#define TELEMETRY_MAX_LEN       60u

/* Field ranges after scaling. Values outside are clamped to the limit. */
#define TELEMETRY_ALT_DM_MIN    (-9999)   /* -999.9 m */
#define TELEMETRY_ALT_DM_MAX    99999     /* 9999.9 m */
#define TELEMETRY_ANGLE_DD_MIN  (-1800)   /* -180.0 deg */
#define TELEMETRY_ANGLE_DD_MAX  1800      /* +180.0 deg */
#define TELEMETRY_BATT_MV_MAX   65535     /* 65.535 V */

/* One packet's worth of data, in engineering units. Set a float field to
 * NAN when there's no valid reading; it's sent as an empty field. */
typedef struct {
    uint16_t    seq;             /* packet counter, wraps 65535 -> 0 */
    uint32_t    time_ms;         /* ms since boot */
    FlightState state;
    float       altitude_m;      /* above the launch site */
    float       max_altitude_m;  /* highest so far since launch; NAN before */
    float       pitch_deg;       /* tilt from vertical, pitch axis */
    float       yaw_deg;         /* tilt from vertical, yaw axis */
    float       battery_v;
} TelemetryFrame;

/* Writes the packet for `f` into `out` as a NUL-terminated string and
 * returns its length. `out_size` must be at least TELEMETRY_MAX_LEN + 1;
 * if it isn't, nothing is written and 0 is returned. */
size_t telemetry_encode(const TelemetryFrame *f, char *out, size_t out_size);

/* CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) of `len` bytes. This is
 * the packet checksum, taken over everything between '$' and '*'. */
uint16_t telemetry_crc16(const char *data, size_t len);

/* When to send: every TELEMETRY_PERIOD_MS, plus straight away whenever the
 * flight state changes (so short states like APOGEE are always sent). */
typedef struct {
    bool        started;       /* false until the first packet */
    uint32_t    last_send_ms;
    FlightState last_state;
} TelemetrySchedule;

/* Clears the schedule so the next check sends. */
void telemetry_schedule_init(TelemetrySchedule *s);

/* Returns true if a packet should be sent now, and if so records it as
 * sent. Call once per loop with the current time and flight state. */
bool telemetry_due(TelemetrySchedule *s, uint32_t now_ms, FlightState state);

#endif /* TVR1_TELEMETRY_H */
