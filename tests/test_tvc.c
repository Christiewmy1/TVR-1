/*
 * Tests for running the gimbal PIDs in BOOST only (tvc.c).
 *
 * The MATLAB match test reuses tests/data/pid_test_vectors.csv: the vectors
 * give the error, and since error = 0 - angle, the angle fed in is -error.
 *
 * Run from the repo root (make test does this) so the CSV path resolves.
 */
#include <math.h>
#include <stdio.h>

#include "unity.h"
#include "config.h"
#include "tvc.h"

#define VECTORS_PATH     "tests/data/pid_test_vectors.csv"
#define VECTORS_ROWS     2001
#define DT               0.001f
#define MATCH_TOLERANCE  1e-4f

static TvcController tvc;
static float pitch_cmd, yaw_cmd;

void setUp(void)
{
    tvc_init(&tvc);
}

void tearDown(void) {}

static void step(FlightState state, float pitch_deg, float yaw_deg)
{
    tvc_update(&tvc, state, pitch_deg, yaw_deg, DT, &pitch_cmd, &yaw_cmd);
}

void test_config_direction_sign_is_plus_or_minus_one(void)
{
    TEST_ASSERT_EQUAL_FLOAT(1.0f, fabsf(GIMBAL_DIRECTION_SIGN));
}

void test_gimbal_centred_outside_boost(void)
{
    const FlightState others[] = { STATE_PAD, STATE_COAST, STATE_APOGEE,
                                   STATE_DESCENT, STATE_LANDED };
    for (size_t i = 0; i < sizeof others / sizeof others[0]; i++) {
        step(others[i], 3.0f, -2.0f);
        TEST_ASSERT_EQUAL_FLOAT(0.0f, pitch_cmd);
        TEST_ASSERT_EQUAL_FLOAT(0.0f, yaw_cmd);
    }
    /* The PIDs never ran. */
    TEST_ASSERT_EQUAL_FLOAT(0.0f, tvc.pitch.integral);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, tvc.yaw.prev_error);
}

void test_pids_reset_on_entering_boost(void)
{
    /* Leftover state from before launch must not carry into the burn. */
    tvc.pitch.integral = 99.0f;
    tvc.pitch.prev_error = -7.0f;
    step(STATE_PAD, 0.0f, 0.0f);

    /* Small tilt so the first-step spike stays inside the 5 deg cone. */
    step(STATE_BOOST, 0.01f, 0.0f);

    PidController fresh;
    pid_init(&fresh, PID_KP, PID_KI, PID_KD);
    float expected = pid_update(&fresh, -0.01f, DT);
    TEST_ASSERT_EQUAL_FLOAT(GIMBAL_DIRECTION_SIGN * expected, pitch_cmd);
    TEST_ASSERT_EQUAL_FLOAT(-0.01f * DT, tvc.pitch.integral);
}

void test_error_is_zero_minus_angle(void)
{
    /* A small positive tilt gives a negative command (before the sign). */
    step(STATE_BOOST, 0.0f, 0.0f);   /* enter BOOST, prev_error = 0 */
    step(STATE_BOOST, 0.001f, 0.0f);  /* tiny step, stays inside the cone */
    TEST_ASSERT_LESS_THAN_FLOAT(0.0f, pitch_cmd * GIMBAL_DIRECTION_SIGN);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, yaw_cmd);
}

void test_large_tilt_is_cone_limited(void)
{
    step(STATE_BOOST, 30.0f, -40.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, GIMBAL_MAX_DEFLECTION_DEG,
                             sqrtf(pitch_cmd * pitch_cmd + yaw_cmd * yaw_cmd));
}

void test_gimbal_centres_as_soon_as_boost_ends(void)
{
    step(STATE_BOOST, 2.0f, 1.5f);
    step(STATE_BOOST, 2.0f, 1.5f);
    TEST_ASSERT_TRUE(pitch_cmd != 0.0f);
    step(STATE_COAST, 2.0f, 1.5f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pitch_cmd);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, yaw_cmd);
}

void test_nan_angle_centres_gimbal_and_leaves_pids_alone(void)
{
    step(STATE_BOOST, 1.0f, 1.0f);
    PidController before = tvc.pitch;

    step(STATE_BOOST, NAN, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pitch_cmd);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, yaw_cmd);
    TEST_ASSERT_EQUAL_FLOAT(before.integral, tvc.pitch.integral);
    TEST_ASSERT_EQUAL_FLOAT(before.prev_error, tvc.pitch.prev_error);
}

void test_boost_matches_matlab_test_vectors(void)
{
    FILE *f = fopen(VECTORS_PATH, "r");
    TEST_ASSERT_NOT_NULL_MESSAGE(f, "can't open " VECTORS_PATH " (run from the repo root)");

    step(STATE_PAD, 0.0f, 0.0f);
    float t, err_p, err_y, ref_p, ref_y, max_err = 0.0f;
    int rows = 0;
    while (fscanf(f, "%f,%f,%f,%f,%f", &t, &err_p, &err_y, &ref_p, &ref_y) == 5) {
        step(STATE_BOOST, -err_p, -err_y);  /* error = 0 - angle */
        max_err = fmaxf(max_err, fabsf(pitch_cmd - GIMBAL_DIRECTION_SIGN * ref_p));
        max_err = fmaxf(max_err, fabsf(yaw_cmd - GIMBAL_DIRECTION_SIGN * ref_y));
        rows++;
    }
    fclose(f);

    printf("  %d rows, max |C - MATLAB| = %.3g deg\n", rows, (double)max_err);
    TEST_ASSERT_EQUAL_INT(VECTORS_ROWS, rows);
    TEST_ASSERT_LESS_THAN_FLOAT(MATCH_TOLERANCE, max_err);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_config_direction_sign_is_plus_or_minus_one);
    RUN_TEST(test_gimbal_centred_outside_boost);
    RUN_TEST(test_pids_reset_on_entering_boost);
    RUN_TEST(test_error_is_zero_minus_angle);
    RUN_TEST(test_large_tilt_is_cone_limited);
    RUN_TEST(test_gimbal_centres_as_soon_as_boost_ends);
    RUN_TEST(test_nan_angle_centres_gimbal_and_leaves_pids_alone);
    RUN_TEST(test_boost_matches_matlab_test_vectors);
    return UNITY_END();
}
