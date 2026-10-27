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

/* ---- Radio ---------------------------------------------------------------
 * Largest payload the LoRa module accepts in one AT send command, in bytes.
 * Every telemetry packet (TELEMETRY_MAX_LEN) must fit in it. */
#define RADIO_MAX_PAYLOAD_BYTES           240u  /* TODO(placeholder): confirm LoRa module with Yusuf */

/* ---- Flight state machine ----------------------------------------------
 * *_HOLD_MS = how long a condition must hold, unbroken, before the state
 * changes. Values from the team lead (2026-10-21), based on the G61W motor;
 * may change once the IMU and barometer are chosen. */

/* PAD -> BOOST: axial acceleration above this (g). Motor gives ~6-7 g. */
#define LAUNCH_ACCEL_THRESHOLD_G          3.0f
#define LAUNCH_HOLD_MS                    50u

/* BOOST -> COAST: axial acceleration below this (g), and at least
 * BURNOUT_MIN_MS_AFTER_LAUNCH since launch so thrust-curve dips can't
 * trigger it. After burnout drag makes the reading slightly negative.
 * Revisit once we have real IMU data. */
#define BURNOUT_ACCEL_THRESHOLD_G         0.0f
#define BURNOUT_HOLD_MS                   50u
#define BURNOUT_MIN_MS_AFTER_LAUNCH       1000u

/* Backup: if still in BOOST this long after launch detect, force COAST
 * (nominal burn is 2.04 s). Covers a missed burnout, e.g. a failed IMU. */
#define BOOST_TIMEOUT_MS                  3000u

/* COAST -> APOGEE: altitude at least this far (m) below its running max */
#define APOGEE_ALT_DROP_M                 2.0f
#define APOGEE_HOLD_MS                    100u

/* DESCENT -> LANDED: altitude changes less than this (m) over the hold
 * time, with acceleration magnitude within the tolerance of 1 g. */
#define LANDED_ALT_BAND_M                 1.0f
#define LANDED_HOLD_MS                    2000u
#define LANDED_ACCEL_TOLERANCE_G          0.2f  /* TODO(placeholder): agreed with team lead; tune once IMU noise is known */

/* A gap of more than this (ms) between usable samples resets any hold in
 * progress; shorter glitches don't. */
#define SENSOR_DROPOUT_RESET_MS           100u

/* ---- Gimbal PID ---------------------------------------------------------
 * Ported unchanged from Simulation/PID_sim_TwoAxis.m (team lead confirmed).
 * The MATLAB notes call these starting-point gains, not yet tuned. Same
 * gains on both axes. The loop's dt is passed in at run time, since the
 * real rate depends on the IMU (the sim uses 1 kHz). */
#define PID_KP                            0.8f
#define PID_KI                            0.05f
#define PID_KD                            0.15f

/* Physical gimbal limit: max deflection from centreline in any direction
 * (a cone, not a per-axis box), in degrees. */
#define GIMBAL_MAX_DEFLECTION_DEG         5.0f

/* +1.0f or -1.0f: flips both gimbal commands if the mechanical "positive"
 * direction turns out to be the opposite of the sim's. */
#define GIMBAL_DIRECTION_SIGN             1.0f  /* TODO(placeholder): Mechanical to confirm which gimbal direction is positive */

#endif /* TVR1_CONFIG_H */
