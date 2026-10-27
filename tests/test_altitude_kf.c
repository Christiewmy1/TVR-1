/*
 * Tests for the altitude Kalman filter (altitude_kf.c), ported from Ethan's
 * DvKalman.m.
 *
 * - The first step is checked against values worked out by hand from the
 *   MATLAB equations (dt = 0.02 s as in the .m file).
 * - The flight replay feeds the updated-airframe OpenRocket altitude with
 *   the real time between rows and checks the velocity estimate against
 *   OpenRocket's vertical velocity. Tolerances are for this sim with the
 *   current noise settings; retuning Q/R may need them revisited.
 *
 * A match against MATLAB's own output needs Ethan to export it (see the PR).
 * Run from the repo root (make test does this) so the CSV path resolves.
 */
#include <math.h>
#include <stdio.h>

#include "unity.h"
#include "config.h"
#include "altitude_kf.h"

#define FLIGHT_PATH     "tests/data/Sensor_Fusion_OR_Data_updated.csv"
#define FLIGHT_ROWS     680
#define OR_APOGEE_S     7.64f

static AltitudeKf kf;

void setUp(void)
{
    alt_kf_init(&kf);
}

void tearDown(void) {}

void test_init_matches_matlab_first_run(void)
{
    TEST_ASSERT_EQUAL_FLOAT(ALT_KF_INIT_POS_M, kf.pos);
    TEST_ASSERT_EQUAL_FLOAT(ALT_KF_INIT_VEL_MPS, kf.vel);
    TEST_ASSERT_EQUAL_FLOAT(ALT_KF_P0, kf.P[0][0]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, kf.P[0][1]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, kf.P[1][0]);
    TEST_ASSERT_EQUAL_FLOAT(ALT_KF_P0, kf.P[1][1]);
}

void test_first_step_matches_hand_calculation(void)
{
    /* x0 = [0 20], P0 = 5I, Q = diag(1,5), R = 10, dt = 0.02, z = 0:
     * xp = [0.4 20], Pp = [6.002 0.1; 0.1 10], K = [6.002 0.1]/16.002 */
    alt_kf_update(&kf, 0.0f, 0.02f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.2499688f, kf.pos);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 19.9975003f, kf.vel);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 3.7507812f, kf.P[0][0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0624922f, kf.P[0][1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0624922f, kf.P[1][0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 9.9993751f, kf.P[1][1]);
}

void test_missing_reading_only_predicts(void)
{
    alt_kf_update(&kf, NAN, 0.05f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f + 0.05f * 20.0f, kf.pos);  /* moved by vel * dt */
    TEST_ASSERT_EQUAL_FLOAT(20.0f, kf.vel);
    /* Uncertainty grows with no measurement: 5 + 0.05^2*5 + 1 */
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 6.0125f, kf.P[0][0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 10.0f, kf.P[1][1]);
}

void test_dt_is_used_not_hardcoded(void)
{
    AltitudeKf slow, fast;
    alt_kf_init(&slow);
    alt_kf_init(&fast);
    alt_kf_update(&slow, NAN, 0.02f);
    alt_kf_update(&fast, NAN, 0.05f);
    TEST_ASSERT_EQUAL_FLOAT(0.4f, slow.pos);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, fast.pos);
}

void test_steady_altitude_settles_with_zero_velocity(void)
{
    for (int i = 0; i < 2000; i++) {
        alt_kf_update(&kf, 100.0f, 0.02f);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, kf.pos);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, kf.vel);
}

void test_flight_replay_velocity_tracks_openrocket(void)
{
    FILE *f = fopen(FLIGHT_PATH, "r");
    TEST_ASSERT_NOT_NULL_MESSAGE(f, "can't open " FLIGHT_PATH " (run from the repo root)");

    char line[512];
    float t, alt, vel_true, prev_t = -1.0f, apogee_cross_s = -1.0f;
    float worst_descent_err = 0.0f, prev_vel = 0.0f;
    int rows = 0;
    while (fgets(line, sizeof line, f) != NULL) {
        if (line[0] == '#' || sscanf(line, "%f,%f,%f", &t, &alt, &vel_true) != 3) {
            continue;
        }
        float dt = (prev_t < 0.0f) ? 0.02f : t - prev_t;  /* first row: no previous time */
        prev_t = t;
        alt_kf_update(&kf, alt, dt);

        if (apogee_cross_s < 0.0f && t > 1.0f && prev_vel > 0.0f && kf.vel <= 0.0f) {
            apogee_cross_s = t;
        }
        if (t > 15.0f) {  /* steady descent */
            worst_descent_err = fmaxf(worst_descent_err, fabsf(kf.vel - vel_true));
        }
        prev_vel = kf.vel;
        rows++;
    }
    fclose(f);

    printf("  velocity crosses 0 at %.3f s (OpenRocket apogee %.3f s); "
           "worst descent velocity error %.2f m/s\n",
           (double)apogee_cross_s, (double)OR_APOGEE_S, (double)worst_descent_err);
    TEST_ASSERT_EQUAL_INT(FLIGHT_ROWS, rows);
    TEST_ASSERT_GREATER_THAN_FLOAT(OR_APOGEE_S, apogee_cross_s);
    TEST_ASSERT_LESS_THAN_FLOAT(OR_APOGEE_S + 1.0f, apogee_cross_s);
    TEST_ASSERT_LESS_THAN_FLOAT(1.0f, worst_descent_err);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_matches_matlab_first_run);
    RUN_TEST(test_first_step_matches_hand_calculation);
    RUN_TEST(test_missing_reading_only_predicts);
    RUN_TEST(test_dt_is_used_not_hardcoded);
    RUN_TEST(test_steady_altitude_settles_with_zero_velocity);
    RUN_TEST(test_flight_replay_velocity_tracks_openrocket);
    return UNITY_END();
}
