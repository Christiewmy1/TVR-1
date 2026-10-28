/*
 * altitude_kf.h - altitude/vertical-velocity Kalman filter, ported from
 * Ethan's DvKalman.m.
 *
 * Two states, position (altitude, m) and velocity (m/s), with a
 * constant-velocity model:
 *
 *   predict:  x = A x,  P = A P A' + Q,     A = [1 dt; 0 1]
 *             Q = q [dt^3/3, dt^2/2; dt^2/2, dt]
 *   update:   K = P H' / (H P H' + R),      H = [1 0]
 *             x = x + K (z - H x),  P = P - K H P
 *
 * z is the measured altitude. The maths is identical to Ethan's
 * DvKalman.m (Simulation/SensorFusion/). The one difference: it starts at
 * the first barometer reading instead of 0 m (zero velocity in both),
 * which is the same thing when altitude is measured from the pad.
 */
#ifndef TVR1_ALTITUDE_KF_H
#define TVR1_ALTITUDE_KF_H

typedef struct {
    float pos;      /* altitude estimate, m */
    float vel;      /* vertical velocity estimate, m/s (+ve up) */
    float P[2][2];  /* error covariance */
} AltitudeKf;

/* Starts the filter at the first valid barometer reading, at rest:
 * position = first_altitude_m, velocity = 0, P = ALT_KF_P0 * I. Call it
 * once that reading arrives, then alt_kf_update() for every later one. */
void alt_kf_init(AltitudeKf *kf, float first_altitude_m);

/* One filter step: predict over dt seconds (> 0), then correct with the
 * measured altitude z (m). If z is NaN (no valid barometer reading), only
 * the prediction is done. */
void alt_kf_update(AltitudeKf *kf, float z, float dt);

#endif /* TVR1_ALTITUDE_KF_H */
