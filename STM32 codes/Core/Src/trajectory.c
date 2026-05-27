#include "trajectory.h"
#include <math.h>

static float maxf(float a, float b)
{
    return (a > b) ? a : b;
}

float trajectory_min_duration(float delta,
                              float max_velocity,
                              float max_acceleration,
                              float max_jerk,
                              scurve_mode_t mode)
{
    const float d = fabsf(delta);
    float t_min = 0.0f;

    if (d <= 0.0f) {
        return 0.0f;
    }

    if (max_velocity > 0.0f) {
        t_min = maxf(t_min, d * scurve_peak_velocity_norm(mode) / max_velocity);
    }

    if (max_acceleration > 0.0f) {
        const float peak_acc = scurve_peak_acceleration_norm(mode);
        if (peak_acc > 0.0f) {
            t_min = maxf(t_min, sqrtf(d * peak_acc / max_acceleration));
        }
    }

    if (max_jerk > 0.0f) {
        const float peak_jerk = scurve_peak_jerk_norm(mode);
        if (peak_jerk > 0.0f) {
            t_min = maxf(t_min, cbrtf(d * peak_jerk / max_jerk));
        }
    }

    return t_min;
}

void trajectory_init(trajectory_t *traj,
                     float start,
                     float target,
                     float duration,
                     scurve_mode_t mode)
{
    if (traj == 0) {
        return;
    }

    traj->start = start;
    traj->target = target;
    traj->duration = duration;
    traj->elapsed = 0.0f;
    traj->position = start;
    traj->velocity = 0.0f;
    traj->acceleration = 0.0f;
    traj->jerk = 0.0f;
    traj->mode = mode;
    traj->active = true;

    if (duration <= 0.0f || start == target) {
        traj->duration = 0.0f;
        traj->position = target;
        traj->active = false;
    }
}

void trajectory_init_limited(trajectory_t *traj,
                             float start,
                             float target,
                             float requested_duration,
                             float max_velocity,
                             float max_acceleration,
                             float max_jerk,
                             scurve_mode_t mode)
{
    const float delta = target - start;
    const float limited_duration = trajectory_min_duration(delta,
                                                           max_velocity,
                                                           max_acceleration,
                                                           max_jerk,
                                                           mode);
    const float duration = maxf(requested_duration, limited_duration);
    trajectory_init(traj, start, target, duration, mode);
}

void trajectory_sample(trajectory_t *traj, float elapsed_time)
{
    if (traj == 0) {
        return;
    }

    if (traj->duration <= 0.0f) {
        traj->elapsed = 0.0f;
        traj->position = traj->target;
        traj->velocity = 0.0f;
        traj->acceleration = 0.0f;
        traj->jerk = 0.0f;
        traj->active = false;
        return;
    }

    traj->elapsed = elapsed_time;
    const float s = scurve_clamp01(traj->elapsed / traj->duration);
    const float delta = traj->target - traj->start;

    const float p = scurve_eval(s, traj->mode);
    const float v_norm = scurve_eval_velocity_norm(s, traj->mode);
    const float a_norm = scurve_eval_acceleration_norm(s, traj->mode);
    const float j_norm = scurve_eval_jerk_norm(s, traj->mode);

    traj->position = traj->start + delta * p;
    traj->velocity = delta * v_norm / traj->duration;
    traj->acceleration = delta * a_norm / (traj->duration * traj->duration);
    traj->jerk = delta * j_norm / (traj->duration * traj->duration * traj->duration);

    if (s >= 1.0f) {
        traj->elapsed = traj->duration;
        traj->position = traj->target;
        traj->velocity = 0.0f;
        if (traj->mode == SCURVE_QUINTIC_SMOOTHERSTEP || traj->mode == SCURVE_LINEAR) {
            traj->acceleration = 0.0f;
        }
        traj->active = false;
    } else {
        traj->active = true;
    }
}

float trajectory_update(trajectory_t *traj, float dt)
{
    if (traj == 0) {
        return 0.0f;
    }

    if (!traj->active) {
        return traj->position;
    }

    trajectory_sample(traj, traj->elapsed + dt);
    return traj->position;
}

void trajectory_retarget_from_current(trajectory_t *traj,
                                      float new_target,
                                      float duration,
                                      scurve_mode_t mode)
{
    if (traj == 0) {
        return;
    }

    trajectory_init(traj, traj->position, new_target, duration, mode);
}

bool trajectory_is_done(const trajectory_t *traj)
{
    if (traj == 0) {
        return true;
    }
    return !traj->active;
}
