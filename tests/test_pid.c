/*
 * Tests for the gimbal PID controller (pid.c).
 *
 * Unit tests check each term and the cone limit by hand. The match test
 * feeds the team lead's MATLAB test vectors (tests/data/pid_test_vectors.csv)
 * through two PIDs + the cone limit and checks every output against MATLAB.
 *
 * Run from the repo root (make test does this) so the CSV path resolves.
 */
#include <math.h>
#include <stdio.h>

#include "unity.h"
#include "config.h"
#include "pid.h"

#define VECTORS_PATH       "tests/data/pid_test_vectors.csv"
#define VECTORS_ROWS       2001
#define VECTORS_DT         0.001f   /* step used to make the vectors */
#define MATCH_TOLERANCE    1e-4f    /* degrees */

static PidController pid;

void setUp(void) {}
void tearDown(void) {}

/* ---- PID terms ---------------------------------------------------------- */

void test_proportional_only_is_kp_times_error(void)
{
    pid_init(&pid, 0.8f, 0.0f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(-1.6f, pid_update(&pid, -2.0f, 0.001f));
    TEST_ASSERT_EQUAL_FLOAT(0.4f, pid_update(&pid, 0.5f, 0.001f));
}

void test_integral_accumulates_error_times_dt(void)
{
    pid_init(&pid, 0.0f, 1.0f, 0.0f);
    float out = 0.0f;
    for (int i = 0; i < 10; i++) {
        out = pid_update(&pid, 2.0f, 0.01f);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.2f, out);  /* 10 x 2.0 x 0.01 */
}

void test_dt_is_used_not_hardcoded(void)
{
    PidController slow, fast;
    pid_init(&slow, 0.0f, 1.0f, 0.0f);
    pid_init(&fast, 0.0f, 1.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.02f, pid_update(&slow, 2.0f, 0.01f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.002f, pid_update(&fast, 2.0f, 0.001f));
}

void test_first_update_derivative_spikes_from_zero_prev_error(void)
{
    /* Matches MATLAB: prev_error starts at 0, so a step of -2 deg at
     * dt = 1 ms gives a derivative of -2000 deg/s. */
    pid_init(&pid, 0.0f, 0.0f, 0.15f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -300.0f, pid_update(&pid, -2.0f, 0.001f));
    /* Same error again: derivative back to 0. */
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, pid_update(&pid, -2.0f, 0.001f));
}

void test_no_anti_windup_integral_keeps_growing(void)
{
    /* Matches MATLAB: the integral isn't held back while the output is
     * beyond the gimbal limit. */
    pid_init(&pid, 0.0f, 1.0f, 0.0f);
    for (int i = 0; i < 1000; i++) {
        pid_update(&pid, 100.0f, 0.01f);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 1000.0f, pid.integral);
}

void test_two_controllers_keep_separate_state(void)
{
    PidController pitch, yaw;
    pid_init(&pitch, PID_KP, PID_KI, PID_KD);
    pid_init(&yaw, PID_KP, PID_KI, PID_KD);
    pid_update(&pitch, -2.0f, 0.001f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, yaw.integral);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, yaw.prev_error);
}

/* ---- cone limit --------------------------------------------------------- */

void test_cone_limit_leaves_small_commands_alone(void)
{
    float p = 3.0f, y = -2.0f;
    gimbal_cone_limit(&p, &y, 5.0f);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, p);
    TEST_ASSERT_EQUAL_FLOAT(-2.0f, y);
}

void test_cone_limit_leaves_command_exactly_at_limit_alone(void)
{
    float p = 3.0f, y = 4.0f;  /* length exactly 5 */
    gimbal_cone_limit(&p, &y, 5.0f);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, p);
    TEST_ASSERT_EQUAL_FLOAT(4.0f, y);
}

void test_cone_limit_scales_both_axes_keeping_direction(void)
{
    /* 4 + 4 would pass a 5 deg box per axis but is 5.66 deg in reality. */
    float p = 4.0f, y = -4.0f;
    gimbal_cone_limit(&p, &y, 5.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 5.0f / sqrtf(2.0f), p);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -5.0f / sqrtf(2.0f), y);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 5.0f, sqrtf(p * p + y * y));
}

void test_cone_limit_single_axis(void)
{
    float p = -12.0f, y = 0.0f;
    gimbal_cone_limit(&p, &y, 5.0f);
    TEST_ASSERT_EQUAL_FLOAT(-5.0f, p);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, y);
}

/* ---- match against MATLAB ----------------------------------------------- */

void test_matches_matlab_test_vectors(void)
{
    FILE *f = fopen(VECTORS_PATH, "r");
    TEST_ASSERT_NOT_NULL_MESSAGE(f, "can't open " VECTORS_PATH " (run from the repo root)");

    PidController pitch, yaw;
    pid_init(&pitch, PID_KP, PID_KI, PID_KD);
    pid_init(&yaw, PID_KP, PID_KI, PID_KD);

    float t, err_p, err_y, ref_p, ref_y;
    float max_err = 0.0f, max_err_t = 0.0f;
    int rows = 0;

    while (fscanf(f, "%f,%f,%f,%f,%f", &t, &err_p, &err_y, &ref_p, &ref_y) == 5) {
        float out_p = pid_update(&pitch, err_p, VECTORS_DT);
        float out_y = pid_update(&yaw, err_y, VECTORS_DT);
        gimbal_cone_limit(&out_p, &out_y, GIMBAL_MAX_DEFLECTION_DEG);

        float e = fmaxf(fabsf(out_p - ref_p), fabsf(out_y - ref_y));
        if (e > max_err) {
            max_err = e;
            max_err_t = t;
        }
        rows++;
    }
    fclose(f);

    printf("  %d rows, max |C - MATLAB| = %.3g deg at t = %.3f s (tolerance %.0e)\n",
           rows, (double)max_err, (double)max_err_t, (double)MATCH_TOLERANCE);
    TEST_ASSERT_EQUAL_INT_MESSAGE(VECTORS_ROWS, rows, "didn't parse every row");
    TEST_ASSERT_LESS_THAN_FLOAT(MATCH_TOLERANCE, max_err);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_proportional_only_is_kp_times_error);
    RUN_TEST(test_integral_accumulates_error_times_dt);
    RUN_TEST(test_dt_is_used_not_hardcoded);
    RUN_TEST(test_first_update_derivative_spikes_from_zero_prev_error);
    RUN_TEST(test_no_anti_windup_integral_keeps_growing);
    RUN_TEST(test_two_controllers_keep_separate_state);
    RUN_TEST(test_cone_limit_leaves_small_commands_alone);
    RUN_TEST(test_cone_limit_leaves_command_exactly_at_limit_alone);
    RUN_TEST(test_cone_limit_scales_both_axes_keeping_direction);
    RUN_TEST(test_cone_limit_single_axis);
    RUN_TEST(test_matches_matlab_test_vectors);
    return UNITY_END();
}
