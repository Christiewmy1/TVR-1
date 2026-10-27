/*
 * tvc.h - thrust vector control: runs the two gimbal PIDs during BOOST only.
 *
 * Call tvc_update() once per control step with the flight state and the
 * rocket's pitch/yaw tilt from vertical (degrees, from sensor fusion). It
 * returns the gimbal deflection commands in degrees:
 *
 *  - Outside BOOST: both commands are 0 (gimbal centred); the PIDs don't run.
 *  - On entering BOOST: both PIDs are reset (integral and previous error).
 *  - In BOOST: error = 0 - angle (as in PID_sim_TwoAxis.m), one PID per
 *    axis, the GIMBAL_MAX_DEFLECTION_DEG cone limit, then
 *    GIMBAL_DIRECTION_SIGN.
 *  - If either angle is NaN (no valid attitude), both commands are 0 for
 *    that step and the PIDs are left untouched.
 */
#ifndef TVR1_TVC_H
#define TVR1_TVC_H

#include "flight_state.h"
#include "pid.h"

typedef struct {
    PidController pitch;
    PidController yaw;
    FlightState   prev_state;  /* state at the previous update */
} TvcController;

/* Sets up both PIDs with the gains from config.h, starting on the pad. */
void tvc_init(TvcController *tvc);

/* One control step; see the rules above. dt is the time since the last
 * step in seconds (> 0). */
void tvc_update(TvcController *tvc, FlightState state,
                float pitch_deg, float yaw_deg, float dt,
                float *pitch_cmd_deg, float *yaw_cmd_deg);

#endif /* TVR1_TVC_H */
