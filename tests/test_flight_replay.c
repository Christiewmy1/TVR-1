/*
 * Replays the OpenRocket flight (tests/data/openrocket_flight.csv) through
 * the state machine and checks each transition against the sim's events.
 *
 * Converting sim data to sensor readings:
 *  - OpenRocket's vertical acceleration excludes gravity; an accelerometer
 *    reads it plus 1 g (1 g sitting on the pad). The rocket is assumed to
 *    stay near vertical, so axial acceleration ~= vertical acceleration.
 *  - The sim ends at ground hit, so a few seconds of lying still on the
 *    ground (1 g, same altitude) are added after the last row.
 *
 * Run from the repo root (make test does this) so the CSV path resolves.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"
#include "config.h"
#include "flight_state.h"

#define CSV_PATH            "tests/data/openrocket_flight.csv"
#define CSV_DATA_ROWS       795
#define G_MPS2              9.81f  /* same g as the thrust-to-weight sheet */
#define GROUND_STEP_MS      50u    /* sample period for the added ground time */
#define GROUND_TIME_MS      3000u
#define NOT_REACHED         UINT32_MAX

/* OpenRocket event times, read from the CSV's "# Event" comment lines. */
typedef struct {
    uint32_t launch_ms;
    uint32_t burnout_ms;
    uint32_t apogee_ms;
    uint32_t ground_hit_ms;
} SimEvents;

static FlightStateMachine sm;
static SimEvents events;
static uint32_t entered_ms[STATE_LANDED + 1];
static uint32_t last_sample_ms;

void setUp(void)
{
    flight_sm_init(&sm);
    for (int s = 0; s <= STATE_LANDED; s++) {
        entered_ms[s] = NOT_REACHED;
    }
    events.launch_ms = events.burnout_ms = NOT_REACHED;
    events.apogee_ms = events.ground_hit_ms = NOT_REACHED;
}

void tearDown(void) {}

static uint32_t to_ms(float seconds)
{
    return (uint32_t)(seconds * 1000.0f + 0.5f);
}

static void feed(const SensorData *d)
{
    FlightState before = sm.state;
    FlightState after = update_state(&sm, d);
    if (after != before) {
        TEST_ASSERT_EQUAL_INT_MESSAGE(before + 1, after, "skipped or went back a state");
        entered_ms[after] = d->time_ms;
    }
    last_sample_ms = d->time_ms;
}

static void record_event(const char *name, float t_s)
{
    if (strcmp(name, "LAUNCH") == 0)     events.launch_ms = to_ms(t_s);
    if (strcmp(name, "BURNOUT") == 0)    events.burnout_ms = to_ms(t_s);
    if (strcmp(name, "APOGEE") == 0)     events.apogee_ms = to_ms(t_s);
    if (strcmp(name, "GROUND_HIT") == 0) events.ground_hit_ms = to_ms(t_s);
}

static void replay(void)
{
    FILE *f = fopen(CSV_PATH, "r");
    TEST_ASSERT_NOT_NULL_MESSAGE(f, "can't open " CSV_PATH " (run from the repo root)");

    char line[256];
    char name[32];
    float t, alt, vel, accel, accel_total;
    float last_alt = 0.0f;
    int rows = 0;

    while (fgets(line, sizeof line, f) != NULL) {
        if (sscanf(line, "# Event %31s occurred at t=%f", name, &t) == 2) {
            record_event(name, t);
        } else if (line[0] != '#' &&
                   sscanf(line, "%f,%f,%f,%f,%f", &t, &alt, &vel, &accel, &accel_total) == 5) {
            float accel_g = (accel + G_MPS2) / G_MPS2;
            SensorData d = {
                .time_ms = to_ms(t),
                .accel_axial_g = accel_g,
                .accel_mag_g = fabsf(accel_g),
                .altitude_m = alt,
                .accel_valid = true,
                .baro_valid = true,
            };
            feed(&d);
            last_alt = alt;
            rows++;
        }
    }
    fclose(f);
    TEST_ASSERT_EQUAL_INT_MESSAGE(CSV_DATA_ROWS, rows, "didn't parse every CSV row");

    /* Lying still on the ground after the sim ends. */
    uint32_t end_ms = last_sample_ms;
    for (uint32_t dt = GROUND_STEP_MS; dt <= GROUND_TIME_MS; dt += GROUND_STEP_MS) {
        SensorData d = {
            .time_ms = end_ms + dt,
            .accel_axial_g = 1.0f,
            .accel_mag_g = 1.0f,
            .altitude_m = last_alt,
            .accel_valid = true,
            .baro_valid = true,
        };
        feed(&d);
    }
}

void test_replay_reaches_every_state_in_order(void)
{
    replay();

    printf("  state     detected   OpenRocket event\n");
    printf("  BOOST   %8.3f s   LAUNCH     %6.3f s\n", entered_ms[STATE_BOOST] / 1000.0, events.launch_ms / 1000.0);
    printf("  COAST   %8.3f s   BURNOUT    %6.3f s\n", entered_ms[STATE_COAST] / 1000.0, events.burnout_ms / 1000.0);
    printf("  APOGEE  %8.3f s   APOGEE     %6.3f s\n", entered_ms[STATE_APOGEE] / 1000.0, events.apogee_ms / 1000.0);
    printf("  DESCENT %8.3f s\n", entered_ms[STATE_DESCENT] / 1000.0);
    printf("  LANDED  %8.3f s   GROUND_HIT %6.3f s\n", entered_ms[STATE_LANDED] / 1000.0, events.ground_hit_ms / 1000.0);

    TEST_ASSERT_EQUAL_INT(STATE_LANDED, sm.state);
}

void test_replay_launch_detected_within_100ms_of_ignition(void)
{
    replay();
    TEST_ASSERT_UINT32_WITHIN(50u, events.launch_ms + 50u, entered_ms[STATE_BOOST]);
}

void test_replay_burnout_detected_after_min_time_and_not_after_motor_burnout(void)
{
    /* Fires once the axial reading drops below BURNOUT_ACCEL_THRESHOLD_G.
     * In this sim the thrust tails off below drag (reading < 0 g) at
     * about 1.74 s, before the motor's own burnout event. */
    replay();
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(entered_ms[STATE_BOOST] + BURNOUT_MIN_MS_AFTER_LAUNCH,
                                        entered_ms[STATE_COAST]);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(events.burnout_ms, entered_ms[STATE_COAST]);
}

void test_replay_apogee_detected_within_1s_after_true_apogee(void)
{
    /* Can't fire before the true peak: the altitude has to drop first. */
    replay();
    TEST_ASSERT_GREATER_THAN_UINT32(events.apogee_ms, entered_ms[STATE_APOGEE]);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(events.apogee_ms + 1000u, entered_ms[STATE_APOGEE]);
}

void test_replay_descent_follows_apogee_on_next_sample(void)
{
    replay();
    TEST_ASSERT_GREATER_THAN_UINT32(entered_ms[STATE_APOGEE], entered_ms[STATE_DESCENT]);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(entered_ms[STATE_APOGEE] + 100u, entered_ms[STATE_DESCENT]);
}

void test_replay_landed_detected_within_hold_time_of_ground_hit(void)
{
    replay();
    TEST_ASSERT_GREATER_THAN_UINT32(events.ground_hit_ms, entered_ms[STATE_LANDED]);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(events.ground_hit_ms + LANDED_HOLD_MS + GROUND_STEP_MS,
                                     entered_ms[STATE_LANDED]);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_replay_reaches_every_state_in_order);
    RUN_TEST(test_replay_launch_detected_within_100ms_of_ignition);
    RUN_TEST(test_replay_burnout_detected_after_min_time_and_not_after_motor_burnout);
    RUN_TEST(test_replay_apogee_detected_within_1s_after_true_apogee);
    RUN_TEST(test_replay_descent_follows_apogee_on_next_sample);
    RUN_TEST(test_replay_landed_detected_within_hold_time_of_ground_hit);
    return UNITY_END();
}
