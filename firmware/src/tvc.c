/*
 * tvc.c - runs the gimbal PIDs during BOOST only. See tvc.h.
 */
#include "tvc.h"

#include <math.h>

#include "config.h"

static void reset_pids(TvcController *tvc)
{
    pid_init(&tvc->pitch, PID_KP, PID_KI, PID_KD);
    pid_init(&tvc->yaw, PID_KP, PID_KI, PID_KD);
}

void tvc_init(TvcController *tvc)
{
    reset_pids(tvc);
    tvc->prev_state = STATE_PAD;
}

void tvc_update(TvcController *tvc, FlightState state,
                float pitch_deg, float yaw_deg, float dt,
                float *pitch_cmd_deg, float *yaw_cmd_deg)
{
    bool entering_boost = (state == STATE_BOOST && tvc->prev_state != STATE_BOOST);
    tvc->prev_state = state;

    *pitch_cmd_deg = 0.0f;
    *yaw_cmd_deg = 0.0f;
    if (state != STATE_BOOST) {
        return;
    }
    if (entering_boost) {
        reset_pids(tvc);
    }
    if (isnan(pitch_deg) || isnan(yaw_deg)) {
        return;
    }

    float pitch_cmd = pid_update(&tvc->pitch, 0.0f - pitch_deg, dt);
    float yaw_cmd = pid_update(&tvc->yaw, 0.0f - yaw_deg, dt);
    gimbal_cone_limit(&pitch_cmd, &yaw_cmd, GIMBAL_MAX_DEFLECTION_DEG);

    *pitch_cmd_deg = GIMBAL_DIRECTION_SIGN * pitch_cmd;
    *yaw_cmd_deg = GIMBAL_DIRECTION_SIGN * yaw_cmd;
}
