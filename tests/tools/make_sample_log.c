/*
 * Builds a sample raw log for the ground station from an OpenRocket flight,
 * using the real flight software: the state machine decides the state and
 * telemetry_encode() builds every packet. Lines look like what the dongle
 * prints (REYAX-style +RCV, see ground/ground_station/radio.py), with some
 * deliberate faults mixed in so a replay exercises the drop counting:
 *
 *   seq % 97  == 13  packet never arrives          -> "missed on air"
 *   seq % 50  == 7   one digit corrupted           -> dropped: checksum
 *   seq % 150 == 75  line cut off before RSSI      -> dropped: rx_format
 *   seq % 120 == 60  module status line "+OK" too  -> ignored
 *
 * RSSI/SNR, battery voltage and receive times are made up. Pitch is the
 * sim's tilt from vertical; yaw is sent as 0.
 *
 * Usage: make sample-log  (writes ground/tests/data/sample_flight_raw.csv)
 * Test tool, not flight code: stdio is fine here.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "flight_state.h"
#include "telemetry.h"

#define SEND_PERIOD_MS   100u            /* 10 Hz; real rate TBD */
#define GROUND_TIME_MS   4000u           /* lying still after the sim ends */
#define GROUND_STEP_MS   50u
#define G_MPS2           9.81f
#define RX_EPOCH_S       1793000000.0    /* fixed so the output never changes */

static FlightStateMachine sm;
static uint16_t seq;
static uint32_t next_send_ms;

static void emit(uint32_t t_ms, float alt, float tilt_deg)
{
    TelemetryFrame frame = {
        .seq = seq,
        .time_ms = t_ms,
        .state = sm.state,
        .altitude_m = alt,
        .pitch_deg = tilt_deg,
        .yaw_deg = 0.0f,
        .battery_v = 7.4f - 0.002f * (float)t_ms / 1000.0f,
    };
    char packet[TELEMETRY_MAX_LEN + 1u];
    size_t len = telemetry_encode(&frame, packet, sizeof packet);
    double rx = RX_EPOCH_S + t_ms / 1000.0 + 0.012;
    int rssi = -30 - (int)(alt / 5.0f);
    float snr = 9.5f - alt / 100.0f;

    if (seq % 97u == 13u) {
        /* lost on air: nothing printed */
    } else if (seq % 50u == 7u) {
        char *last_digit = &packet[len - 6u];  /* battery's last digit, before "*XXXX" */
        *last_digit = (char)(*last_digit == '9' ? '0' : *last_digit + 1);
        printf("%.3f,\"+RCV=1,%zu,%s,%d,%.1f\"\n", rx, len, packet, rssi, (double)snr);
    } else if (seq % 150u == 75u) {
        printf("%.3f,\"+RCV=1,%zu,%s\"\n", rx, len, packet);
    } else {
        printf("%.3f,\"+RCV=1,%zu,%s,%d,%.1f\"\n", rx, len, packet, rssi, (double)snr);
    }
    if (seq % 120u == 60u) {
        printf("%.3f,\"+OK\"\n", rx + 0.001);
    }
    seq++;
}

static void step(uint32_t t_ms, float accel_g, float alt, float tilt_deg)
{
    SensorData d = {
        .time_ms = t_ms,
        .accel_axial_g = accel_g,
        .accel_mag_g = fabsf(accel_g),
        .altitude_m = alt,
        .accel_valid = true,
        .baro_valid = true,
    };
    update_state(&sm, &d);
    if (t_ms >= next_send_ms) {
        emit(t_ms, alt, tilt_deg);
        next_send_ms += SEND_PERIOD_MS;
    }
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        fprintf(stderr, "usage: %s <openrocket_13_column.csv>\n", argv[0]);
        return 2;
    }
    FILE *f = fopen(argv[1], "r");
    if (f == NULL) {
        perror(argv[1]);
        return 1;
    }

    flight_sm_init(&sm);
    printf("rx_time_s,line\n");

    char line[512];
    float t, alt = 0.0f, vel, accel, accel_total, roll, pitch_rate, yaw_rate, zenith = 90.0f;
    uint32_t t_ms = 0u;
    while (fgets(line, sizeof line, f) != NULL) {
        if (line[0] != '#' &&
            sscanf(line, "%f,%f,%f,%f,%f,%f,%f,%f,%f", &t, &alt, &vel, &accel,
                   &accel_total, &roll, &pitch_rate, &yaw_rate, &zenith) == 9) {
            t_ms = (uint32_t)(t * 1000.0f + 0.5f);
            step(t_ms, (accel + G_MPS2) / G_MPS2, alt, 90.0f - zenith);
        }
    }
    fclose(f);

    for (uint32_t dt = GROUND_STEP_MS; dt <= GROUND_TIME_MS; dt += GROUND_STEP_MS) {
        step(t_ms + dt, 1.0f, alt, 90.0f - zenith);
    }
    return 0;
}
