#ifndef SCURVE_H
#define SCURVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/**
 * @brief Available normalized interpolation profiles.
 *
 * Input s is normalized progress from 0 to 1.
 * Output p is shaped progress from 0 to 1.
 */
typedef enum {
    SCURVE_LINEAR = 0,
    SCURVE_CUBIC_SMOOTHSTEP,
    SCURVE_QUINTIC_SMOOTHERSTEP
} scurve_mode_t;

/** Clamp a floating point value to [0, 1]. */
float scurve_clamp01(float x);

/** Evaluate normalized position shape p(s). */
float scurve_eval(float s, scurve_mode_t mode);

/** Evaluate dp/ds for the normalized profile. */
float scurve_eval_velocity_norm(float s, scurve_mode_t mode);

/** Evaluate d^2p/ds^2 for the normalized profile. */
float scurve_eval_acceleration_norm(float s, scurve_mode_t mode);

/** Evaluate d^3p/ds^3 for the normalized profile. */
float scurve_eval_jerk_norm(float s, scurve_mode_t mode);

/** Return peak normalized |dp/ds| for a profile. */
float scurve_peak_velocity_norm(scurve_mode_t mode);

/** Return peak normalized |d^2p/ds^2| for a profile. */
float scurve_peak_acceleration_norm(scurve_mode_t mode);

/** Return peak normalized |d^3p/ds^3| for a profile. */
float scurve_peak_jerk_norm(scurve_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif /* SCURVE_H */
