/*
 * Unit tests for the flight state machine (flight_state.c).
 *
 * Synthetic samples built from the thresholds in config.h, fed on a test
 * clock that ticks DT_MS per sample. They test the logic (hold times,
 * forward-only, sensor dropout), not the threshold values, so they keep
 * passing if config.h changes. The replay of the OpenRocket flight is in
 * test_flight_replay.c.
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "unity.h"
#include "config.h"
#include "flight_state.h"

#define DT_MS  10u  /* test sample period; every *_HOLD_MS must be a multiple */

/* Samples needed for a condition to hold for hold_ms: the first one starts
 * the hold, the last one is hold_ms later. */
#define SAMPLES_FOR(hold_ms)  ((hold_ms) / DT_MS + 1u)

/* Readings safely on each side of the thresholds. */
#define ACCEL_REST_G    1.0f
#define ACCEL_BOOST_G   (LAUNCH_ACCEL_THRESHOLD_G + 1.0f)
#define ACCEL_COAST_G   (BURNOUT_ACCEL_THRESHOLD_G - 1.0f)
#define PEAK_ALT_M      100.0f
#define BELOW_PEAK_M    (PEAK_ALT_M - APOGEE_ALT_DROP_M - 1.0f)

static FlightStateMachine sm;
static uint32_t clock_ms;

void setUp(void)
{
    flight_sm_init(&sm);
    clock_ms = 0u;
}

void tearDown(void) {}

static SensorData sample(float accel_g, float altitude_m)
{
    SensorData d = {
        .time_ms = 0u,
        .accel_axial_g = accel_g,
        .accel_mag_g = fabsf(accel_g),
        .altitude_m = altitude_m,
        .accel_valid = true,
        .baro_valid = true,
    };
    return d;
}

/* Feeds `d` n times, one every DT_MS; returns the state after the last. */
static FlightState feed(SensorData d, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        d.time_ms = clock_ms;
        update_state(&sm, &d);
        clock_ms += DT_MS;
    }
    return sm.state;
}

/* Drive the machine through real transitions to reach a given state. */
static void reach_boost(void)
{
    TEST_ASSERT_EQUAL_INT(STATE_BOOST,
                          feed(sample(ACCEL_BOOST_G, 0.0f), SAMPLES_FOR(LAUNCH_HOLD_MS)));
}

static void reach_coast(void)
{
    reach_boost();
    feed(sample(ACCEL_BOOST_G, PEAK_ALT_M), BURNOUT_MIN_MS_AFTER_LAUNCH / DT_MS);
    TEST_ASSERT_EQUAL_INT(STATE_COAST,
                          feed(sample(ACCEL_COAST_G, PEAK_ALT_M), SAMPLES_FOR(BURNOUT_HOLD_MS)));
}

static void reach_descent(void)
{
    reach_coast();
    TEST_ASSERT_EQUAL_INT(STATE_APOGEE,
                          feed(sample(ACCEL_COAST_G, BELOW_PEAK_M), SAMPLES_FOR(APOGEE_HOLD_MS)));
    TEST_ASSERT_EQUAL_INT(STATE_DESCENT, feed(sample(ACCEL_COAST_G, BELOW_PEAK_M), 1u));
}

/* ---- config sanity ------------------------------------------------------ */

void test_config_every_transition_needs_a_hold_time(void)
{
    TEST_ASSERT_GREATER_THAN_UINT32(0u, LAUNCH_HOLD_MS);
    TEST_ASSERT_GREATER_THAN_UINT32(0u, BURNOUT_HOLD_MS);
    TEST_ASSERT_GREATER_THAN_UINT32(0u, APOGEE_HOLD_MS);
    TEST_ASSERT_GREATER_THAN_UINT32(0u, LANDED_HOLD_MS);
}

void test_config_launch_threshold_is_above_rest_and_burnout(void)
{
    TEST_ASSERT_GREATER_THAN_FLOAT(ACCEL_REST_G, LAUNCH_ACCEL_THRESHOLD_G);
    TEST_ASSERT_GREATER_THAN_FLOAT(BURNOUT_ACCEL_THRESHOLD_G, LAUNCH_ACCEL_THRESHOLD_G);
}

/* ---- PAD -> BOOST ------------------------------------------------------- */

void test_starts_on_pad(void)
{
    TEST_ASSERT_EQUAL_INT(STATE_PAD, sm.state);
}

void test_stays_on_pad_at_rest(void)
{
    TEST_ASSERT_EQUAL_INT(STATE_PAD, feed(sample(ACCEL_REST_G, 0.0f), 1000u));
}

void test_launch_needs_full_hold_time(void)
{
    TEST_ASSERT_EQUAL_INT(STATE_PAD,
                          feed(sample(ACCEL_BOOST_G, 0.0f), SAMPLES_FOR(LAUNCH_HOLD_MS) - 1u));
    TEST_ASSERT_EQUAL_INT(STATE_BOOST, feed(sample(ACCEL_BOOST_G, 0.0f), 1u));
}

void test_single_accel_spike_does_not_launch(void)
{
    feed(sample(ACCEL_BOOST_G, 0.0f), 1u);
    TEST_ASSERT_EQUAL_INT(STATE_PAD, feed(sample(ACCEL_REST_G, 0.0f), 10u));
}

void test_interrupted_launch_restarts_hold(void)
{
    feed(sample(ACCEL_BOOST_G, 0.0f), SAMPLES_FOR(LAUNCH_HOLD_MS) - 1u);
    feed(sample(ACCEL_REST_G, 0.0f), 1u);
    TEST_ASSERT_EQUAL_INT(STATE_PAD,
                          feed(sample(ACCEL_BOOST_G, 0.0f), SAMPLES_FOR(LAUNCH_HOLD_MS) - 1u));
}

void test_short_accel_dropout_does_not_break_launch_hold(void)
{
    SensorData dropout = sample(ACCEL_REST_G, 0.0f);
    dropout.accel_valid = false;

    feed(sample(ACCEL_BOOST_G, 0.0f), SAMPLES_FOR(LAUNCH_HOLD_MS) - 2u);
    TEST_ASSERT_EQUAL_INT(STATE_PAD, feed(dropout, 1u));
    TEST_ASSERT_EQUAL_INT(STATE_BOOST, feed(sample(ACCEL_BOOST_G, 0.0f), 1u));
}

void test_dropout_gap_of_exactly_the_limit_keeps_launch_hold(void)
{
    SensorData dropout = sample(ACCEL_REST_G, 0.0f);
    dropout.accel_valid = false;

    feed(sample(ACCEL_BOOST_G, 0.0f), SAMPLES_FOR(LAUNCH_HOLD_MS) - 1u);
    /* Next valid sample lands exactly SENSOR_DROPOUT_RESET_MS later. */
    feed(dropout, SENSOR_DROPOUT_RESET_MS / DT_MS - 1u);
    TEST_ASSERT_EQUAL_INT(STATE_BOOST, feed(sample(ACCEL_BOOST_G, 0.0f), 1u));
}

void test_long_accel_dropout_restarts_launch_hold(void)
{
    SensorData dropout = sample(ACCEL_REST_G, 0.0f);
    dropout.accel_valid = false;

    feed(sample(ACCEL_BOOST_G, 0.0f), SAMPLES_FOR(LAUNCH_HOLD_MS) - 1u);
    /* Next valid sample lands one tick past SENSOR_DROPOUT_RESET_MS. */
    feed(dropout, SENSOR_DROPOUT_RESET_MS / DT_MS);
    TEST_ASSERT_EQUAL_INT(STATE_PAD,
                          feed(sample(ACCEL_BOOST_G, 0.0f), SAMPLES_FOR(LAUNCH_HOLD_MS) - 1u));
    TEST_ASSERT_EQUAL_INT(STATE_BOOST, feed(sample(ACCEL_BOOST_G, 0.0f), 1u));
}

/* ---- BOOST -> COAST ----------------------------------------------------- */

void test_burnout_ignored_in_first_second_then_needs_full_hold(void)
{
    reach_boost();
    /* Low accel from right after launch: too early to count. */
    TEST_ASSERT_EQUAL_INT(STATE_BOOST,
                          feed(sample(ACCEL_COAST_G, 50.0f),
                               BURNOUT_MIN_MS_AFTER_LAUNCH / DT_MS - 1u));
    /* Now past the minimum time: the hold starts here. */
    TEST_ASSERT_EQUAL_INT(STATE_BOOST,
                          feed(sample(ACCEL_COAST_G, 50.0f), SAMPLES_FOR(BURNOUT_HOLD_MS) - 1u));
    TEST_ASSERT_EQUAL_INT(STATE_COAST, feed(sample(ACCEL_COAST_G, 50.0f), 1u));
}

void test_single_low_accel_during_boost_is_ignored(void)
{
    reach_boost();
    feed(sample(ACCEL_BOOST_G, 50.0f), BURNOUT_MIN_MS_AFTER_LAUNCH / DT_MS);
    feed(sample(ACCEL_COAST_G, 50.0f), 1u);
    TEST_ASSERT_EQUAL_INT(STATE_BOOST, feed(sample(ACCEL_BOOST_G, 50.0f), 10u));
}

/* ---- COAST -> APOGEE -> DESCENT ----------------------------------------- */

void test_still_climbing_is_not_apogee(void)
{
    reach_coast();
    float alt = PEAK_ALT_M;
    for (uint32_t i = 0; i < 100u; i++) {
        alt += 1.0f;
        feed(sample(ACCEL_COAST_G, alt), 1u);
    }
    TEST_ASSERT_EQUAL_INT(STATE_COAST, sm.state);
}

void test_small_drop_is_not_apogee(void)
{
    reach_coast();
    TEST_ASSERT_EQUAL_INT(STATE_COAST,
                          feed(sample(ACCEL_COAST_G, PEAK_ALT_M - APOGEE_ALT_DROP_M + 0.5f), 100u));
}

void test_drop_of_exactly_the_threshold_is_apogee(void)
{
    reach_coast();
    TEST_ASSERT_EQUAL_INT(STATE_APOGEE,
                          feed(sample(ACCEL_COAST_G, PEAK_ALT_M - APOGEE_ALT_DROP_M),
                               SAMPLES_FOR(APOGEE_HOLD_MS)));
}

void test_single_baro_glitch_is_not_apogee(void)
{
    reach_coast();
    feed(sample(ACCEL_COAST_G, BELOW_PEAK_M), 1u);
    TEST_ASSERT_EQUAL_INT(STATE_COAST, feed(sample(ACCEL_COAST_G, PEAK_ALT_M), 20u));
}

void test_baro_dropout_is_not_apogee(void)
{
    SensorData dropout = sample(ACCEL_COAST_G, 0.0f);
    dropout.baro_valid = false;

    reach_coast();
    TEST_ASSERT_EQUAL_INT(STATE_COAST, feed(dropout, 100u));
}

void test_sustained_drop_is_apogee_then_descent(void)
{
    reach_coast();
    TEST_ASSERT_EQUAL_INT(STATE_COAST,
                          feed(sample(ACCEL_COAST_G, BELOW_PEAK_M), SAMPLES_FOR(APOGEE_HOLD_MS) - 1u));
    TEST_ASSERT_EQUAL_INT(STATE_APOGEE, feed(sample(ACCEL_COAST_G, BELOW_PEAK_M), 1u));
    TEST_ASSERT_EQUAL_INT(STATE_DESCENT, feed(sample(ACCEL_COAST_G, BELOW_PEAK_M), 1u));
}

/* ---- DESCENT -> LANDED -------------------------------------------------- */

void test_slow_steady_descent_is_not_landed(void)
{
    /* 1 m/s at 1 g: each step is tiny, but over the hold time the altitude
     * moves more than the band. */
    reach_descent();
    float alt = BELOW_PEAK_M;
    for (uint32_t i = 0; i < 2u * SAMPLES_FOR(LANDED_HOLD_MS); i++) {
        alt -= 1.0f * (float)DT_MS / 1000.0f;
        feed(sample(ACCEL_REST_G, alt), 1u);
    }
    TEST_ASSERT_EQUAL_INT(STATE_DESCENT, sm.state);
}

void test_steady_altitude_at_rest_is_landed(void)
{
    reach_descent();
    TEST_ASSERT_EQUAL_INT(STATE_DESCENT,
                          feed(sample(ACCEL_REST_G, 3.0f), SAMPLES_FOR(LANDED_HOLD_MS) - 1u));
    TEST_ASSERT_EQUAL_INT(STATE_LANDED, feed(sample(ACCEL_REST_G, 3.0f), 1u));
}

void test_lying_on_its_side_is_landed(void)
{
    /* Long axis horizontal: axial reads ~0 g, but the magnitude is 1 g. */
    SensorData on_side = sample(0.0f, 3.0f);
    on_side.accel_mag_g = 1.0f;

    reach_descent();
    TEST_ASSERT_EQUAL_INT(STATE_LANDED, feed(on_side, SAMPLES_FOR(LANDED_HOLD_MS)));
}

void test_long_dropout_restarts_landed_window(void)
{
    SensorData dropout = sample(ACCEL_REST_G, 3.0f);
    dropout.baro_valid = false;

    reach_descent();
    feed(sample(ACCEL_REST_G, 3.0f), SAMPLES_FOR(LANDED_HOLD_MS) - 1u);
    feed(dropout, SENSOR_DROPOUT_RESET_MS / DT_MS);
    TEST_ASSERT_EQUAL_INT(STATE_DESCENT,
                          feed(sample(ACCEL_REST_G, 3.0f), SAMPLES_FOR(LANDED_HOLD_MS) - 1u));
    TEST_ASSERT_EQUAL_INT(STATE_LANDED, feed(sample(ACCEL_REST_G, 3.0f), 1u));
}

void test_steady_altitude_without_1g_is_not_landed(void)
{
    reach_descent();
    TEST_ASSERT_EQUAL_INT(STATE_DESCENT,
                          feed(sample(1.0f + 2.0f * LANDED_ACCEL_TOLERANCE_G, 3.0f),
                               2u * SAMPLES_FOR(LANDED_HOLD_MS)));
}

/* ---- forward only ------------------------------------------------------- */

void test_coast_does_not_go_back_to_boost(void)
{
    reach_coast();
    TEST_ASSERT_EQUAL_INT(STATE_COAST, feed(sample(ACCEL_BOOST_G, PEAK_ALT_M + 10.0f), 100u));
}

void test_landed_is_final(void)
{
    reach_descent();
    feed(sample(ACCEL_REST_G, 3.0f), SAMPLES_FOR(LANDED_HOLD_MS));
    TEST_ASSERT_EQUAL_INT(STATE_LANDED, sm.state);
    TEST_ASSERT_EQUAL_INT(STATE_LANDED, feed(sample(ACCEL_BOOST_G, 500.0f), 100u));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_config_every_transition_needs_a_hold_time);
    RUN_TEST(test_config_launch_threshold_is_above_rest_and_burnout);
    RUN_TEST(test_starts_on_pad);
    RUN_TEST(test_stays_on_pad_at_rest);
    RUN_TEST(test_launch_needs_full_hold_time);
    RUN_TEST(test_single_accel_spike_does_not_launch);
    RUN_TEST(test_interrupted_launch_restarts_hold);
    RUN_TEST(test_short_accel_dropout_does_not_break_launch_hold);
    RUN_TEST(test_dropout_gap_of_exactly_the_limit_keeps_launch_hold);
    RUN_TEST(test_long_accel_dropout_restarts_launch_hold);
    RUN_TEST(test_burnout_ignored_in_first_second_then_needs_full_hold);
    RUN_TEST(test_single_low_accel_during_boost_is_ignored);
    RUN_TEST(test_still_climbing_is_not_apogee);
    RUN_TEST(test_small_drop_is_not_apogee);
    RUN_TEST(test_drop_of_exactly_the_threshold_is_apogee);
    RUN_TEST(test_single_baro_glitch_is_not_apogee);
    RUN_TEST(test_baro_dropout_is_not_apogee);
    RUN_TEST(test_sustained_drop_is_apogee_then_descent);
    RUN_TEST(test_slow_steady_descent_is_not_landed);
    RUN_TEST(test_steady_altitude_at_rest_is_landed);
    RUN_TEST(test_lying_on_its_side_is_landed);
    RUN_TEST(test_long_dropout_restarts_landed_window);
    RUN_TEST(test_steady_altitude_without_1g_is_not_landed);
    RUN_TEST(test_coast_does_not_go_back_to_boost);
    RUN_TEST(test_landed_is_final);
    return UNITY_END();
}
