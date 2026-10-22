/*
 * config.h - every threshold, gain, limit and timing used by the flight
 * software, as named constants in one place.
 *
 * Values not yet confirmed by the team are marked like this:
 *
 *   #define LAUNCH_ACCEL_THRESHOLD_G  3.0f  // TODO(placeholder): confirm with team lead
 *
 * Search for "TODO(placeholder)" to find every unconfirmed value. None of
 * them may remain before flight.
 *
 * Constants are added together with the module that uses them.
 */
#ifndef TVR1_CONFIG_H
#define TVR1_CONFIG_H

/* ---- Flight state machine ----------------------------------------------
 * *_HOLD_MS = how long a condition must hold, unbroken, before the state
 * changes. Values from the team lead (2026-10-21), based on the G61W motor;
 * may change once the IMU and barometer are chosen. */

/* PAD -> BOOST: axial acceleration above this (g). Motor gives ~6-7 g. */
#define LAUNCH_ACCEL_THRESHOLD_G          3.0f
#define LAUNCH_HOLD_MS                    50u

/* BOOST -> COAST: axial acceleration below this (g), and at least
 * BURNOUT_MIN_MS_AFTER_LAUNCH since launch so thrust-curve dips can't
 * trigger it. */
#define BURNOUT_ACCEL_THRESHOLD_G         1.0f
#define BURNOUT_HOLD_MS                   50u
#define BURNOUT_MIN_MS_AFTER_LAUNCH       1000u

/* COAST -> APOGEE: altitude at least this far (m) below its running max */
#define APOGEE_ALT_DROP_M                 2.0f
#define APOGEE_HOLD_MS                    100u

/* DESCENT -> LANDED: altitude changes less than this (m) over the hold
 * time, with acceleration magnitude within the tolerance of 1 g. */
#define LANDED_ALT_BAND_M                 1.0f
#define LANDED_HOLD_MS                    2000u
#define LANDED_ACCEL_TOLERANCE_G          0.2f  /* TODO(placeholder): confirm with team lead */

#endif /* TVR1_CONFIG_H */
