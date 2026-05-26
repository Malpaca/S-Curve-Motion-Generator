#ifndef SCURVE_H
#define SCURVE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SCURVE_LINEAR = 0,
    SCURVE_CUBIC_SMOOTHSTEP,
    SCURVE_QUINTIC_SMOOTHERSTEP
} scurve_mode_t;

float scurve_clamp01(float x);
float scurve_eval(float s, scurve_mode_t mode);
float scurve_eval_velocity_norm(float s, scurve_mode_t mode);
float scurve_eval_acceleration_norm(float s, scurve_mode_t mode);
float scurve_eval_jerk_norm(float s, scurve_mode_t mode);

float scurve_peak_velocity_norm(scurve_mode_t mode);
float scurve_peak_acceleration_norm(scurve_mode_t mode);
float scurve_peak_jerk_norm(scurve_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif
