/*
 * altitude_kf.h - altitude/vertical-velocity Kalman filter, ported from
 * Ethan's DvKalman.m.
 *
 * Two states, position (altitude, m) and velocity (m/s), with a
 * constant-velocity model:
 *
 *   predict:  x = A x,  P = A P A' + Q,     A = [1 dt; 0 1]
 *   update:   K = P H' / (H P H' + R),      H = [1 0]
 *             x = x + K (z - H x),  P = P - K H P
 *
 * z is the measured altitude. Kept identical to the MATLAB so outputs
 * match it; the only change is that dt is a parameter instead of fixed at
 * 0.02 s, because sensor samples won't arrive exactly 20 ms apart.
 */
#ifndef TVR1_ALTITUDE_KF_H
#define TVR1_ALTITUDE_KF_H

typedef struct {
    float pos;      /* altitude estimate, m */
    float vel;      /* vertical velocity estimate, m/s (+ve up) */
    float P[2][2];  /* error covariance */
} AltitudeKf;

/* Starts from ALT_KF_INIT_POS_M / ALT_KF_INIT_VEL_MPS with P = ALT_KF_P0 * I,
 * as DvKalman.m does on its first run. */
void alt_kf_init(AltitudeKf *kf);

/* One filter step: predict over dt seconds (> 0), then correct with the
 * measured altitude z (m). If z is NaN (no valid barometer reading), only
 * the prediction is done. */
void alt_kf_update(AltitudeKf *kf, float z, float dt);

#endif /* TVR1_ALTITUDE_KF_H */
