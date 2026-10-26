/*
 * telemetry.c - telemetry packet encoder. See telemetry.h and
 * docs/packet_spec.md.
 *
 * Builds the text by hand instead of with snprintf: printf-family calls
 * aren't allowed in firmware/src, and this keeps the output identical on
 * every C library.
 */
#include "telemetry.h"

#include <math.h>

typedef struct {
    char   *buf;
    size_t  len;
} Writer;

static void put_char(Writer *w, char c)
{
    w->buf[w->len++] = c;
}

static void put_str(Writer *w, const char *s)
{
    while (*s != '\0') {
        put_char(w, *s++);
    }
}

static void put_uint(Writer *w, uint32_t v)
{
    char digits[10];
    int n = 0;
    do {
        digits[n++] = (char)('0' + v % 10u);
        v /= 10u;
    } while (v != 0u);
    while (n > 0) {
        put_char(w, digits[--n]);
    }
}

static void put_int(Writer *w, int32_t v)
{
    if (v < 0) {
        put_char(w, '-');
        put_uint(w, 0u - (uint32_t)v);
    } else {
        put_uint(w, (uint32_t)v);
    }
}

/* Writes value * scale, rounded to the nearest integer and clamped to
 * [min, max]. NaN writes nothing, leaving an empty field (no reading).
 * Clamping before rounding keeps lroundf() away from huge values. */
static void put_scaled(Writer *w, float value, float scale, int32_t min, int32_t max)
{
    if (isnan(value)) {
        return;
    }
    float scaled = value * scale;
    if (scaled <= (float)min) {
        put_int(w, min);
    } else if (scaled >= (float)max) {
        put_int(w, max);
    } else {
        put_int(w, (int32_t)lroundf(scaled));
    }
}

uint16_t telemetry_crc16(const char *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)((uint8_t)data[i] << 8);
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

size_t telemetry_encode(const TelemetryFrame *f, char *out, size_t out_size)
{
    static const char HEX[] = "0123456789ABCDEF";

    if (out_size < TELEMETRY_MAX_LEN + 1u) {
        return 0u;
    }

    Writer w = { out, 0u };
    put_str(&w, "$TVR,");
    put_uint(&w, TELEMETRY_VERSION);
    put_char(&w, ',');
    put_uint(&w, f->seq);
    put_char(&w, ',');
    put_uint(&w, f->time_ms);
    put_char(&w, ',');
    put_uint(&w, (uint32_t)f->state);
    put_char(&w, ',');
    put_scaled(&w, f->altitude_m, 10.0f, TELEMETRY_ALT_DM_MIN, TELEMETRY_ALT_DM_MAX);
    put_char(&w, ',');
    put_scaled(&w, f->pitch_deg, 10.0f, TELEMETRY_ANGLE_DD_MIN, TELEMETRY_ANGLE_DD_MAX);
    put_char(&w, ',');
    put_scaled(&w, f->yaw_deg, 10.0f, TELEMETRY_ANGLE_DD_MIN, TELEMETRY_ANGLE_DD_MAX);
    put_char(&w, ',');
    put_scaled(&w, f->battery_v, 1000.0f, 0, TELEMETRY_BATT_MV_MAX);

    /* Checksum covers everything after '$', up to (not including) '*'. */
    uint16_t crc = telemetry_crc16(out + 1, w.len - 1u);
    put_char(&w, '*');
    put_char(&w, HEX[(crc >> 12) & 0xFu]);
    put_char(&w, HEX[(crc >> 8) & 0xFu]);
    put_char(&w, HEX[(crc >> 4) & 0xFu]);
    put_char(&w, HEX[crc & 0xFu]);

    out[w.len] = '\0';
    return w.len;
}
