/*
 * telemetry.h - telemetry packet encoder. The format is specified in
 * docs/packet_spec.md (DRAFT until Yusuf and Samantha sign off).
 *
 * A packet is one line of plain ASCII text, e.g.
 *
 *   $TVR,1,42,1234,1,515,20,-15,7312*1A2B
 *
 * so it can go straight into the LoRa module's AT send command. Values are
 * sent as scaled integers (altitude in 0.1 m, angles in 0.1 deg, battery in
 * mV), so no float formatting is needed on the flight computer.
 */
#ifndef TVR1_TELEMETRY_H
#define TVR1_TELEMETRY_H

#include <stddef.h>
#include <stdint.h>

#include "flight_state.h"

#define TELEMETRY_VERSION       1u

/* Longest possible packet, in characters (no terminator): every field at
 * its widest. See docs/packet_spec.md. */
#define TELEMETRY_MAX_LEN       54u

/* Field ranges after scaling. Values outside are clamped to the limit. */
#define TELEMETRY_ALT_DM_MIN    (-9999)   /* -999.9 m */
#define TELEMETRY_ALT_DM_MAX    99999     /* 9999.9 m */
#define TELEMETRY_ANGLE_DD_MIN  (-1800)   /* -180.0 deg */
#define TELEMETRY_ANGLE_DD_MAX  1800      /* +180.0 deg */
#define TELEMETRY_BATT_MV_MAX   65535     /* 65.535 V */

/* One packet's worth of data, in engineering units. Set a float field to
 * NAN when there's no valid reading; it's sent as an empty field. */
typedef struct {
    uint16_t    seq;          /* packet counter, wraps 65535 -> 0 */
    uint32_t    time_ms;      /* ms since boot */
    FlightState state;
    float       altitude_m;   /* above the launch site */
    float       pitch_deg;    /* tilt from vertical, pitch axis */
    float       yaw_deg;      /* tilt from vertical, yaw axis */
    float       battery_v;
} TelemetryFrame;

/* Writes the packet for `f` into `out` as a NUL-terminated string and
 * returns its length. `out_size` must be at least TELEMETRY_MAX_LEN + 1;
 * if it isn't, nothing is written and 0 is returned. */
size_t telemetry_encode(const TelemetryFrame *f, char *out, size_t out_size);

/* CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) of `len` bytes. This is
 * the packet checksum, taken over everything between '$' and '*'. */
uint16_t telemetry_crc16(const char *data, size_t len);

#endif /* TVR1_TELEMETRY_H */
