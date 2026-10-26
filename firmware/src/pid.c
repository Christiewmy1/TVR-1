/*
 * pid.c - gimbal PID controller. See pid.h.
 *
 * Each step mirrors a line of PID_sim_TwoAxis.m, in the same order, so the
 * float results track MATLAB's as closely as possible.
 */
#include "pid.h"

#include <math.h>

void pid_init(PidController *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

float pid_update(PidController *pid, float error, float dt)
{
    pid->integral = pid->integral + error * dt;
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}

void gimbal_cone_limit(float *pitch_deg, float *yaw_deg, float max_deg)
{
    float magnitude = sqrtf(*pitch_deg * *pitch_deg + *yaw_deg * *yaw_deg);
    if (magnitude > max_deg) {
        float scale = max_deg / magnitude;
        *pitch_deg *= scale;
        *yaw_deg *= scale;
    }
}
