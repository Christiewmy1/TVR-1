/*
 * Tests for the telemetry encoder (telemetry.c).
 *
 * The golden vectors (tests/data/telemetry_vectors.csv) are shared with the
 * Python decoder's tests: here we check the C encoder produces each packet
 * exactly; ground/tests/test_packet.py checks Python decodes them back.
 * Their checksums came from Python's binascii.crc_hqx, not from this code.
 *
 * Run from the repo root (make test does this) so the CSV path resolves.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"
#include "config.h"
#include "telemetry.h"

#define VECTORS_PATH  "tests/data/telemetry_vectors.csv"
#define VECTORS_ROWS  8

void setUp(void) {}
void tearDown(void) {}

void test_crc16_standard_check_value(void)
{
    /* Published check value for CRC-16/CCITT-FALSE. */
    TEST_ASSERT_EQUAL_HEX16(0x29B1, telemetry_crc16("123456789", 9));
}

void test_widest_packet_fits_the_radio(void)
{
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(RADIO_MAX_PAYLOAD_BYTES, TELEMETRY_MAX_LEN);
}

void test_state_numbers_match_the_spec(void)
{
    /* docs/packet_spec.md fixes these numbers; reordering the enum would
     * silently change what the ground station shows. */
    TEST_ASSERT_EQUAL_INT(0, STATE_PAD);
    TEST_ASSERT_EQUAL_INT(1, STATE_BOOST);
    TEST_ASSERT_EQUAL_INT(2, STATE_COAST);
    TEST_ASSERT_EQUAL_INT(3, STATE_APOGEE);
    TEST_ASSERT_EQUAL_INT(4, STATE_DESCENT);
    TEST_ASSERT_EQUAL_INT(5, STATE_LANDED);
}

void test_buffer_too_small_writes_nothing(void)
{
    TelemetryFrame f = { .seq = 1u, .time_ms = 2u, .state = STATE_PAD, .altitude_m = 0.0f,
                         .max_altitude_m = NAN, .pitch_deg = 0.0f, .yaw_deg = 0.0f,
                         .battery_v = 7.4f };
    char buf[TELEMETRY_MAX_LEN];  /* one short: no room for the terminator */
    memset(buf, 'x', sizeof buf);

    TEST_ASSERT_EQUAL_size_t(0, telemetry_encode(&f, buf, sizeof buf));
    TEST_ASSERT_EACH_EQUAL_CHAR('x', buf, sizeof buf);
}

void test_packet_is_printable_ascii_with_no_line_breaks(void)
{
    TelemetryFrame f = { .seq = 65535u, .time_ms = 4294967295u, .state = STATE_LANDED,
                         .altitude_m = -999.9f, .max_altitude_m = -999.9f, .pitch_deg = -180.0f,
                         .yaw_deg = -180.0f, .battery_v = 65.535f };
    char buf[TELEMETRY_MAX_LEN + 1u];
    size_t len = telemetry_encode(&f, buf, sizeof buf);

    TEST_ASSERT_EQUAL_size_t(TELEMETRY_MAX_LEN, len);
    for (size_t i = 0; i < len; i++) {
        TEST_ASSERT_TRUE(buf[i] >= 0x20 && buf[i] <= 0x7E);
    }
}

void test_encodes_golden_vectors(void)
{
    FILE *f = fopen(VECTORS_PATH, "r");
    TEST_ASSERT_NOT_NULL_MESSAGE(f, "can't open " VECTORS_PATH " (run from the repo root)");

    char line[256];
    int rows = 0;
    while (fgets(line, sizeof line, f) != NULL) {
        unsigned long seq, time_ms;
        int state;
        float alt, max_alt, pitch, yaw, batt;
        char expected[TELEMETRY_MAX_LEN + 2u];

        if (line[0] == '#' ||
            sscanf(line, "%lu,%lu,%d,%f,%f,%f,%f,%f|%61s",
                   &seq, &time_ms, &state, &alt, &max_alt, &pitch, &yaw, &batt, expected) != 9) {
            continue;
        }
        TelemetryFrame frame = {
            .seq = (uint16_t)seq,
            .time_ms = (uint32_t)time_ms,
            .state = (FlightState)state,
            .altitude_m = alt,
            .max_altitude_m = max_alt,
            .pitch_deg = pitch,
            .yaw_deg = yaw,
            .battery_v = batt,
        };
        char buf[TELEMETRY_MAX_LEN + 1u];
        size_t len = telemetry_encode(&frame, buf, sizeof buf);

        TEST_ASSERT_EQUAL_STRING_MESSAGE(expected, buf, line);
        TEST_ASSERT_EQUAL_size_t(strlen(expected), len);
        rows++;
    }
    fclose(f);
    TEST_ASSERT_EQUAL_INT_MESSAGE(VECTORS_ROWS, rows, "didn't parse every vector");
}

/* ---- send schedule ------------------------------------------------------ */

void test_first_check_sends(void)
{
    TelemetrySchedule s;
    telemetry_schedule_init(&s);
    TEST_ASSERT_TRUE(telemetry_due(&s, 12345u, STATE_PAD));
}

void test_sends_once_per_period(void)
{
    TelemetrySchedule s;
    telemetry_schedule_init(&s);
    telemetry_due(&s, 1000u, STATE_PAD);
    TEST_ASSERT_FALSE(telemetry_due(&s, 1000u + TELEMETRY_PERIOD_MS - 1u, STATE_PAD));
    TEST_ASSERT_TRUE(telemetry_due(&s, 1000u + TELEMETRY_PERIOD_MS, STATE_PAD));
    TEST_ASSERT_FALSE(telemetry_due(&s, 1000u + TELEMETRY_PERIOD_MS + 1u, STATE_PAD));
}

void test_state_change_sends_straight_away(void)
{
    TelemetrySchedule s;
    telemetry_schedule_init(&s);
    telemetry_due(&s, 1000u, STATE_COAST);
    /* APOGEE lasts one sample and DESCENT follows straight after: both go. */
    TEST_ASSERT_TRUE(telemetry_due(&s, 1010u, STATE_APOGEE));
    TEST_ASSERT_TRUE(telemetry_due(&s, 1030u, STATE_DESCENT));
    TEST_ASSERT_FALSE(telemetry_due(&s, 1040u, STATE_DESCENT));
    /* The regular period restarts from the last packet sent. */
    TEST_ASSERT_TRUE(telemetry_due(&s, 1030u + TELEMETRY_PERIOD_MS, STATE_DESCENT));
}

void test_rate_fits_within_ten_hz(void)
{
    /* Team lead: 10 Hz may not fit the LoRa airtime; plan on ~5 Hz. */
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(100u, TELEMETRY_PERIOD_MS);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_first_check_sends);
    RUN_TEST(test_sends_once_per_period);
    RUN_TEST(test_state_change_sends_straight_away);
    RUN_TEST(test_rate_fits_within_ten_hz);
    RUN_TEST(test_crc16_standard_check_value);
    RUN_TEST(test_widest_packet_fits_the_radio);
    RUN_TEST(test_state_numbers_match_the_spec);
    RUN_TEST(test_buffer_too_small_writes_nothing);
    RUN_TEST(test_packet_is_printable_ascii_with_no_line_breaks);
    RUN_TEST(test_encodes_golden_vectors);
    return UNITY_END();
}
