#include "scurve.h"

float scurve_clamp01(float x)
{
    if (x < 0.0f) {
        return 0.0f;
    }
    if (x > 1.0f) {
        return 1.0f;
    }
    return x;
}

float scurve_eval(float s, scurve_mode_t mode)
{
    s = scurve_clamp01(s);

    switch (mode) {
    case SCURVE_LINEAR:
        return s;

    case SCURVE_CUBIC_SMOOTHSTEP:
        /* p(s) = 3s^2 - 2s^3 */
        return (3.0f * s * s) - (2.0f * s * s * s);

    case SCURVE_QUINTIC_SMOOTHERSTEP:
        /* p(s) = 10s^3 - 15s^4 + 6s^5 */
        return (10.0f * s * s * s)
             - (15.0f * s * s * s * s)
             + (6.0f * s * s * s * s * s);

    default:
        return s;
    }
}

float scurve_eval_velocity_norm(float s, scurve_mode_t mode)
{
    s = scurve_clamp01(s);

    switch (mode) {
    case SCURVE_LINEAR:
        return 1.0f;

    case SCURVE_CUBIC_SMOOTHSTEP:
        /* dp/ds = 6s - 6s^2 */
        return (6.0f * s) - (6.0f * s * s);

    case SCURVE_QUINTIC_SMOOTHERSTEP:
        /* dp/ds = 30s^2 - 60s^3 + 30s^4 */
        return (30.0f * s * s)
             - (60.0f * s * s * s)
             + (30.0f * s * s * s * s);

    default:
        return 1.0f;
    }
}

float scurve_eval_acceleration_norm(float s, scurve_mode_t mode)
{
    s = scurve_clamp01(s);

    switch (mode) {
    case SCURVE_LINEAR:
        return 0.0f;

    case SCURVE_CUBIC_SMOOTHSTEP:
        /* d^2p/ds^2 = 6 - 12s */
        return 6.0f - (12.0f * s);

    case SCURVE_QUINTIC_SMOOTHERSTEP:
        /* d^2p/ds^2 = 60s - 180s^2 + 120s^3 */
        return (60.0f * s)
             - (180.0f * s * s)
             + (120.0f * s * s * s);

    default:
        return 0.0f;
    }
}

float scurve_eval_jerk_norm(float s, scurve_mode_t mode)
{
    s = scurve_clamp01(s);

    switch (mode) {
    case SCURVE_LINEAR:
        return 0.0f;

    case SCURVE_CUBIC_SMOOTHSTEP:
        /* d^3p/ds^3 = -12 */
        return -12.0f;

    case SCURVE_QUINTIC_SMOOTHERSTEP:
        /* d^3p/ds^3 = 60 - 360s + 360s^2 */
        return 60.0f - (360.0f * s) + (360.0f * s * s);

    default:
        return 0.0f;
    }
}

float scurve_peak_velocity_norm(scurve_mode_t mode)
{
    switch (mode) {
    case SCURVE_LINEAR:
        return 1.0f;
    case SCURVE_CUBIC_SMOOTHSTEP:
        return 1.5f;
    case SCURVE_QUINTIC_SMOOTHERSTEP:
        return 1.875f;
    default:
        return 1.0f;
    }
}

float scurve_peak_acceleration_norm(scurve_mode_t mode)
{
    switch (mode) {
    case SCURVE_LINEAR:
        return 0.0f;
    case SCURVE_CUBIC_SMOOTHSTEP:
        return 6.0f;
    case SCURVE_QUINTIC_SMOOTHERSTEP:
        /* Exact value is 10 / sqrt(3), approximately 5.77350269. */
        return 5.773503f;
    default:
        return 0.0f;
    }
}

float scurve_peak_jerk_norm(scurve_mode_t mode)
{
    switch (mode) {
    case SCURVE_LINEAR:
        return 0.0f;
    case SCURVE_CUBIC_SMOOTHSTEP:
        return 12.0f;
    case SCURVE_QUINTIC_SMOOTHERSTEP:
        return 60.0f;
    default:
        return 0.0f;
    }
}
