/*
 * Tests for the altitude Kalman filter (altitude_kf.c), ported from Ethan's
 * DvKalman.m.
 *
 * - The first step is checked against values worked out by hand from the
 *   MATLAB equations.
 * - The flight replay feeds the updated-airframe OpenRocket altitude with
 *   the real time between rows and checks the velocity estimate against
 *   OpenRocket's vertical velocity. Tolerances are for this sim with the
 *   current noise settings; retuning Q/R may need them revisited.
 *
 * - The MATLAB match test feeds the same inputs as Ethan's RunFilter.m
 *   (the flight's altitude column, dt = 0.008 s on every step, starting at
 *   0 m) and compares against his output, tests/data/kf_vectors.csv.
 * Run from the repo root (make test does this) so the CSV paths resolve.
 */
#include <math.h>
#include <stdio.h>

#include "unity.h"
#include "config.h"
#include "altitude_kf.h"

#define FLIGHT_PATH     "tests/data/Sensor_Fusion_OR_Data_updated.csv"
#define FLIGHT_ROWS     680
#define OR_APOGEE_S     7.64f
#define VECTORS_PATH    "tests/data/kf_vectors.csv"
#define MATLAB_DT       0.008f   /* RunFilter.m: time(2) - time(1) of the flight file */

static AltitudeKf kf;

void setUp(void)
{
    alt_kf_init(&kf, 0.0f);
}

void tearDown(void) {}

void test_init_starts_at_first_reading_at_rest(void)
{
    alt_kf_init(&kf, 12.5f);
    TEST_ASSERT_EQUAL_FLOAT(12.5f, kf.pos);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, kf.vel);
    TEST_ASSERT_EQUAL_FLOAT(ALT_KF_P0, kf.P[0][0]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, kf.P[0][1]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, kf.P[1][0]);
    TEST_ASSERT_EQUAL_FLOAT(ALT_KF_P0, kf.P[1][1]);
}

void test_first_step_matches_hand_calculation(void)
{
    /* x0 = [10 0], P0 = 5I, R = 10, dt = 0.02, z = 12,
     * Q = [dt^3/3 dt^2/2; dt^2/2 dt] = [2.667e-6 2e-4; 2e-4 0.02]:
     * xp = [10 0], Pp = [5.0020027 0.1002; 0.1002 5.02],
     * K = [5.0020027 0.1002]/15.0020027, innovation = 2 */
    alt_kf_init(&kf, 10.0f);
    alt_kf_update(&kf, 12.0f, 0.02f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 10.6668447f, kf.pos);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0133582f, kf.vel);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 3.3342233f, kf.P[0][0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0667911f, kf.P[0][1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0667911f, kf.P[1][0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 5.0193308f, kf.P[1][1]);
}

void test_missing_reading_only_predicts(void)
{
    alt_kf_init(&kf, 5.0f);
    alt_kf_update(&kf, NAN, 0.05f);
    TEST_ASSERT_EQUAL_FLOAT(5.0f, kf.pos);  /* at rest: prediction doesn't move */
    TEST_ASSERT_EQUAL_FLOAT(0.0f, kf.vel);
    /* Uncertainty grows with no measurement: 5 + 0.05^2*5 + 0.05^3/3, 5 + 0.05 */
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 5.0125417f, kf.P[0][0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 5.05f, kf.P[1][1]);
}

void test_dt_is_used_not_hardcoded(void)
{
    AltitudeKf slow, fast;
    alt_kf_init(&slow, 0.0f);
    alt_kf_init(&fast, 0.0f);
    alt_kf_update(&slow, NAN, 0.02f);
    alt_kf_update(&fast, NAN, 0.05f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 5.0020027f, slow.P[0][0]);  /* 5 + 0.02^2*5 + 0.02^3/3 */
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 5.0125417f, fast.P[0][0]);  /* 5 + 0.05^2*5 + 0.05^3/3 */
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
        if (prev_t < 0.0f) {
            alt_kf_init(&kf, alt);  /* first reading: start there, at rest */
        } else {
            alt_kf_update(&kf, alt, t - prev_t);
        }
        prev_t = t;

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
    /* With Ethan's current Q (dt-based, scale 1) the velocity lags: it
     * crosses 0 about 1.15 s after true apogee (0.60 s with the earlier
     * fixed diag(1,5)). Q will be retuned for real sensor data. */
    TEST_ASSERT_GREATER_THAN_FLOAT(OR_APOGEE_S, apogee_cross_s);
    TEST_ASSERT_LESS_THAN_FLOAT(OR_APOGEE_S + 1.5f, apogee_cross_s);
    TEST_ASSERT_LESS_THAN_FLOAT(1.0f, worst_descent_err);
}

void test_matches_matlab_kf_vectors(void)
{
    FILE *flight = fopen(FLIGHT_PATH, "r");
    FILE *vectors = fopen(VECTORS_PATH, "r");
    TEST_ASSERT_NOT_NULL_MESSAGE(flight, "can't open " FLIGHT_PATH " (run from the repo root)");
    TEST_ASSERT_NOT_NULL_MESSAGE(vectors, "can't open " VECTORS_PATH " (run from the repo root)");

    char line[512];
    float t, alt, ref_pos, ref_vel, ref_p11, ref_p22;
    float max_pos = 0.0f, max_vel = 0.0f, max_p = 0.0f;
    int rows = 0;
    alt_kf_init(&kf, 0.0f);  /* RunFilter.m starts at x = [0 0] */
    while (fgets(line, sizeof line, flight) != NULL) {
        if (line[0] == '#' || sscanf(line, "%f,%f", &t, &alt) != 2) {
            continue;
        }
        TEST_ASSERT_EQUAL_INT(4, fscanf(vectors, "%f,%f,%f,%f",
                                        &ref_pos, &ref_vel, &ref_p11, &ref_p22));
        alt_kf_update(&kf, alt, MATLAB_DT);
        max_pos = fmaxf(max_pos, fabsf(kf.pos - ref_pos));
        max_vel = fmaxf(max_vel, fabsf(kf.vel - ref_vel));
        max_p = fmaxf(max_p, fmaxf(fabsf(kf.P[0][0] - ref_p11), fabsf(kf.P[1][1] - ref_p22)));
        rows++;
    }
    fclose(flight);
    fclose(vectors);

    printf("  %d rows, max |C - MATLAB|: pos %.2g m, vel %.2g m/s, P %.2g\n",
           rows, (double)max_pos, (double)max_vel, (double)max_p);
    TEST_ASSERT_EQUAL_INT(FLIGHT_ROWS, rows);
    TEST_ASSERT_LESS_THAN_FLOAT(1e-3f, max_pos);
    TEST_ASSERT_LESS_THAN_FLOAT(1e-3f, max_vel);
    TEST_ASSERT_LESS_THAN_FLOAT(1e-4f, max_p);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_starts_at_first_reading_at_rest);
    RUN_TEST(test_first_step_matches_hand_calculation);
    RUN_TEST(test_missing_reading_only_predicts);
    RUN_TEST(test_dt_is_used_not_hardcoded);
    RUN_TEST(test_steady_altitude_settles_with_zero_velocity);
    RUN_TEST(test_flight_replay_velocity_tracks_openrocket);
    RUN_TEST(test_matches_matlab_kf_vectors);
    return UNITY_END();
}
