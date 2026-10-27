/*
 * flight_state.c - flight phase state machine. See flight_state.h.
 */
#include "flight_state.h"

#include <float.h>
#include <math.h>

#include "config.h"

/* Call on every usable sample before checking a condition. A gap longer
 * than SENSOR_DROPOUT_RESET_MS since the last one breaks any hold in
 * progress; shorter glitches don't. */
static void note_sample(FlightStateMachine *sm, uint32_t now_ms)
{
    if (sm->holding &&
        (uint32_t)(now_ms - sm->last_sample_ms) > SENSOR_DROPOUT_RESET_MS) {
        sm->holding = false;
    }
    sm->last_sample_ms = now_ms;
}

/* Tracks how long `condition` has held without a break; one valid sample
 * where it doesn't hold resets it. Returns true once it has held for at
 * least `hold_ms`. Unsigned subtraction keeps this right if time_ms wraps. */
static bool held_for(FlightStateMachine *sm, bool condition,
                     uint32_t now_ms, uint32_t hold_ms)
{
    if (!condition) {
        sm->holding = false;
        return false;
    }
    if (!sm->holding) {
        sm->holding = true;
        sm->hold_start_ms = now_ms;
    }
    return (uint32_t)(now_ms - sm->hold_start_ms) >= hold_ms;
}

/* Landed = altitude range stays under LANDED_ALT_BAND_M and acceleration
 * stays near 1 g for LANDED_HOLD_MS. If the altitude moves too far, a new
 * window starts at the current sample. */
static bool landed(FlightStateMachine *sm, const SensorData *d)
{
    bool at_rest = fabsf(d->accel_mag_g - 1.0f) <= LANDED_ACCEL_TOLERANCE_G;

    if (sm->holding) {
        sm->window_min_m = fminf(sm->window_min_m, d->altitude_m);
        sm->window_max_m = fmaxf(sm->window_max_m, d->altitude_m);
        if (sm->window_max_m - sm->window_min_m >= LANDED_ALT_BAND_M) {
            sm->holding = false;
        }
    }
    if (!sm->holding) {
        sm->window_min_m = d->altitude_m;
        sm->window_max_m = d->altitude_m;
    }
    return held_for(sm, at_rest, d->time_ms, LANDED_HOLD_MS);
}

static void enter(FlightStateMachine *sm, FlightState next)
{
    sm->state = next;
    sm->holding = false;
}

void flight_sm_init(FlightStateMachine *sm)
{
    sm->state = STATE_PAD;
    sm->holding = false;
    sm->hold_start_ms = 0u;
    sm->last_sample_ms = 0u;
    sm->launch_ms = 0u;
    sm->boost_timed_out = false;
    sm->max_altitude_m = -FLT_MAX;  /* any real reading replaces this */
    sm->window_min_m = 0.0f;
    sm->window_max_m = 0.0f;
}

FlightState update_state(FlightStateMachine *sm, const SensorData *d)
{
    /* Track peak altitude from launch until apogee. */
    if ((sm->state == STATE_BOOST || sm->state == STATE_COAST) &&
        d->baro_valid && d->altitude_m > sm->max_altitude_m) {
        sm->max_altitude_m = d->altitude_m;
    }

    switch (sm->state) {
    case STATE_PAD:
        if (d->accel_valid) {
            note_sample(sm, d->time_ms);
            if (held_for(sm, d->accel_axial_g > LAUNCH_ACCEL_THRESHOLD_G,
                         d->time_ms, LAUNCH_HOLD_MS)) {
                enter(sm, STATE_BOOST);
                sm->launch_ms = d->time_ms;
            }
        }
        break;

    case STATE_BOOST:
        if (d->accel_valid) {
            note_sample(sm, d->time_ms);
            if (held_for(sm,
                         d->accel_axial_g < BURNOUT_ACCEL_THRESHOLD_G &&
                         (uint32_t)(d->time_ms - sm->launch_ms) >= BURNOUT_MIN_MS_AFTER_LAUNCH,
                         d->time_ms, BURNOUT_HOLD_MS)) {
                enter(sm, STATE_COAST);
            }
        }
        /* Backup, checked on every sample so it works even if the IMU has
         * stopped giving valid readings. */
        if (sm->state == STATE_BOOST &&
            (uint32_t)(d->time_ms - sm->launch_ms) >= BOOST_TIMEOUT_MS) {
            enter(sm, STATE_COAST);
            sm->boost_timed_out = true;
        }
        break;

    case STATE_COAST:
        if (d->baro_valid) {
            note_sample(sm, d->time_ms);
            if (held_for(sm, d->altitude_m <= sm->max_altitude_m - APOGEE_ALT_DROP_M,
                         d->time_ms, APOGEE_HOLD_MS)) {
                enter(sm, STATE_APOGEE);
            }
        }
        break;

    case STATE_APOGEE:
        enter(sm, STATE_DESCENT);
        break;

    case STATE_DESCENT:
        if (d->baro_valid && d->accel_valid) {
            note_sample(sm, d->time_ms);
            if (landed(sm, d)) {
                enter(sm, STATE_LANDED);
            }
        }
        break;

    case STATE_LANDED:
        break;
    }

    return sm->state;
}

float flight_sm_max_altitude(const FlightStateMachine *sm)
{
    if (sm->state == STATE_PAD || sm->max_altitude_m == -FLT_MAX) {
        return NAN;
    }
    return sm->max_altitude_m;
}
