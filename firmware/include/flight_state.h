/*
 * flight_state.h - flight phase state machine.
 *
 * The rocket is always in exactly one FlightState, and states only move
 * forward: PAD -> BOOST -> COAST -> APOGEE -> DESCENT -> LANDED.
 *
 * Each transition needs its condition to hold, unbroken, for a set time
 * (the *_HOLD_MS constants in config.h), so a single noisy reading can't
 * change the state. Times come from SensorData.time_ms, so the logic works
 * at any sample rate.
 */
#ifndef TVR1_FLIGHT_STATE_H
#define TVR1_FLIGHT_STATE_H

#include <stdbool.h>
#include <stdint.h>

#include "sensor_data.h"

typedef enum {
    STATE_PAD, STATE_BOOST, STATE_COAST,
    STATE_APOGEE, STATE_DESCENT, STATE_LANDED
} FlightState;

/* Everything the state machine remembers between samples. The caller owns
 * it; set it up with flight_sm_init() before the first update. */
typedef struct {
    FlightState state;
    bool        holding;         /* current state's exit condition is holding */
    uint32_t    hold_start_ms;   /* when it started holding */
    uint32_t    last_sample_ms;  /* time of the last usable sample */
    uint32_t    launch_ms;       /* when BOOST was entered */
    float       max_altitude_m;  /* highest altitude seen since launch */
    float       window_min_m;    /* altitude range seen in the current */
    float       window_max_m;    /*   "landed" window during descent */
} FlightStateMachine;

/* Puts the state machine on the pad with everything cleared. */
void flight_sm_init(FlightStateMachine *sm);

/* Feeds one sensor sample and returns the state after it. A sample whose
 * needed sensor is marked invalid is skipped. A short dropout doesn't break
 * a hold in progress, but a gap longer than SENSOR_DROPOUT_RESET_MS
 * restarts it. APOGEE lasts exactly one sample, then moves to DESCENT. */
FlightState update_state(FlightStateMachine *sm, const SensorData *d);

#endif /* TVR1_FLIGHT_STATE_H */
