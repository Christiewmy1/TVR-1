/*
 * pid.h - PID controller for the gimbal, ported from
 * Simulation/PID_sim_TwoAxis.m.
 *
 * One PidController per gimbal axis (pitch and yaw). Each update takes the
 * angle error in degrees and returns a deflection command in degrees.
 * gimbal_cone_limit() then caps the two axes' commands together.
 *
 * Kept identical to the MATLAB so the outputs match it. Known behaviour,
 * to be changed later as a separate step:
 *  - no anti-windup: the integral keeps growing while the output is limited
 *  - prev_error starts at 0, so the first update's derivative term spikes
 *    (error / dt); the cone limit clips it
 */
#ifndef TVR1_PID_H
#define TVR1_PID_H

typedef struct {
    float kp, ki, kd;   /* gains */
    float integral;     /* running sum of error * dt */
    float prev_error;   /* error from the previous update */
} PidController;

/* Sets the gains and clears the integral and previous error. */
void pid_init(PidController *pid, float kp, float ki, float kd);

/* Feeds one error sample (degrees) and returns the unlimited command
 * (degrees). dt is the time since the previous update in seconds; it must
 * be greater than 0. */
float pid_update(PidController *pid, float error, float dt);

/* If the combined pitch/yaw command is longer than max_deg, scales both
 * down by the same factor so the direction is kept and the length equals
 * max_deg. Commands within the limit are left unchanged. */
void gimbal_cone_limit(float *pitch_deg, float *yaw_deg, float max_deg);

#endif /* TVR1_PID_H */
