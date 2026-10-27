/*
 * altitude_kf.c - altitude/vertical-velocity Kalman filter. See
 * altitude_kf.h.
 *
 * The 2x2 matrix products are written out by hand (no matrix library),
 * following the MATLAB line by line. P is kept as a full matrix rather than
 * assumed symmetric, so the float results track MATLAB's.
 */
#include "altitude_kf.h"

#include <math.h>

#include "config.h"

void alt_kf_init(AltitudeKf *kf, float first_altitude_m)
{
    kf->pos = first_altitude_m;
    kf->vel = 0.0f;
    kf->P[0][0] = ALT_KF_P0;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = ALT_KF_P0;
}

void alt_kf_update(AltitudeKf *kf, float z, float dt)
{
    /* xp = A*x */
    float xp0 = kf->pos + dt * kf->vel;
    float xp1 = kf->vel;

    /* Pp = A*P*A' + Q */
    float ap00 = kf->P[0][0] + dt * kf->P[1][0];  /* A*P, row 0 */
    float ap01 = kf->P[0][1] + dt * kf->P[1][1];
    float pp00 = ap00 + dt * ap01 + ALT_KF_Q_POS;
    float pp01 = ap01;
    float pp10 = kf->P[1][0] + dt * kf->P[1][1];
    float pp11 = kf->P[1][1] + ALT_KF_Q_VEL;

    if (isnan(z)) {
        kf->pos = xp0;
        kf->vel = xp1;
        kf->P[0][0] = pp00;
        kf->P[0][1] = pp01;
        kf->P[1][0] = pp10;
        kf->P[1][1] = pp11;
        return;
    }

    /* K = Pp*H' * inv(H*Pp*H' + R); with H = [1 0] that's column 0 of Pp
     * divided by a scalar. */
    float s = pp00 + ALT_KF_R;
    float k0 = pp00 / s;
    float k1 = pp10 / s;

    /* x = xp + K*(z - H*xp) */
    float innovation = z - xp0;
    kf->pos = xp0 + k0 * innovation;
    kf->vel = xp1 + k1 * innovation;

    /* P = Pp - K*H*Pp; K*H*Pp = K times row 0 of Pp */
    kf->P[0][0] = pp00 - k0 * pp00;
    kf->P[0][1] = pp01 - k0 * pp01;
    kf->P[1][0] = pp10 - k1 * pp00;
    kf->P[1][1] = pp11 - k1 * pp01;
}
