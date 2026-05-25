#include "trajectory.h"
#include <math.h>

static float max2(float a, float b)
{
    return (a > b) ? a : b;
}

traj_status_t trajectory_init(
    trajectory_t *traj,
    float start,
    float target,
    float duration,
    scurve_mode_t mode
)
{
    if (traj == 0) {
        return TRAJ_STATUS_NULL_POINTER;
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

    if (duration <= 0.0f) {
        traj->position = target;
        traj->active = false;
        return TRAJ_STATUS_INVALID_DURATION;
    }

    return TRAJ_STATUS_OK;
}

float trajectory_min_duration(
    float delta,
    float vmax,
    float amax,
    float jmax,
    scurve_mode_t mode
)
{
    const float d = fabsf(delta);
    float t_min = 0.0f;

    if (d <= 0.0f) {
        return 0.0f;
    }

    if (vmax > 0.0f) {
        const float tv = d * scurve_peak_velocity_norm(mode) / vmax;
        t_min = max2(t_min, tv);
    }

    if (amax > 0.0f) {
        const float a_shape = scurve_peak_acceleration_norm(mode);
        if (a_shape > 0.0f) {
            const float ta = sqrtf(d * a_shape / amax);
            t_min = max2(t_min, ta);
        }
    }

    if (jmax > 0.0f) {
        const float j_shape = scurve_peak_jerk_norm(mode);
        if (j_shape > 0.0f) {
            const float tj = cbrtf(d * j_shape / jmax);
            t_min = max2(t_min, tj);
        }
    }

    return t_min;
}

traj_status_t trajectory_init_limited(
    trajectory_t *traj,
    float start,
    float target,
    float requested_duration,
    float vmax,
    float amax,
    float jmax,
    scurve_mode_t mode
)
{
    if (traj == 0) {
        return TRAJ_STATUS_NULL_POINTER;
    }

    if (requested_duration < 0.0f) {
        requested_duration = 0.0f;
    }

    const float min_duration = trajectory_min_duration(
        target - start,
        vmax,
        amax,
        jmax,
        mode
    );

    const float duration = max2(requested_duration, min_duration);

    if (duration <= 0.0f) {
        return trajectory_init(traj, start, target, 0.0f, mode);
    }

    return trajectory_init(traj, start, target, duration, mode);
}

float trajectory_update(trajectory_t *traj, float dt)
{
    if (traj == 0) {
        return 0.0f;
    }

    if (!traj->active) {
        return traj->position;
    }

    if (dt < 0.0f) {
        dt = 0.0f;
    }

    traj->elapsed += dt;

    float s = 0.0f;
    if (traj->duration > 0.0f) {
        s = scurve_clamp01(traj->elapsed / traj->duration);
    } else {
        s = 1.0f;
    }

    const float delta = traj->target - traj->start;
    const float p = scurve_eval(s, traj->mode);
    const float dp = scurve_eval_velocity_norm(s, traj->mode);
    const float ddp = scurve_eval_acceleration_norm(s, traj->mode);
    const float dddp = scurve_eval_jerk_norm(s, traj->mode);

    traj->position = traj->start + (delta * p);

    if (traj->duration > 0.0f) {
        traj->velocity = delta * dp / traj->duration;
        traj->acceleration = delta * ddp / (traj->duration * traj->duration);
        traj->jerk = delta * dddp / (traj->duration * traj->duration * traj->duration);
    } else {
        traj->velocity = 0.0f;
        traj->acceleration = 0.0f;
        traj->jerk = 0.0f;
    }

    if (s >= 1.0f) {
        traj->position = traj->target;
        traj->velocity = 0.0f;
        if (traj->mode == SCURVE_QUINTIC_SMOOTHERSTEP || traj->mode == SCURVE_LINEAR) {
            traj->acceleration = 0.0f;
        }
        traj->active = false;
    }

    return traj->position;
}

void trajectory_reset(trajectory_t *traj)
{
    if (traj == 0) {
        return;
    }

    traj->elapsed = 0.0f;
    traj->position = traj->start;
    traj->velocity = 0.0f;
    traj->acceleration = 0.0f;
    traj->jerk = 0.0f;
    traj->active = (traj->duration > 0.0f);
}

traj_status_t trajectory_retarget(
    trajectory_t *traj,
    float new_target,
    float duration,
    scurve_mode_t mode
)
{
    if (traj == 0) {
        return TRAJ_STATUS_NULL_POINTER;
    }

    return trajectory_init(traj, traj->position, new_target, duration, mode);
}

bool trajectory_is_done(const trajectory_t *traj)
{
    if (traj == 0) {
        return true;
    }
    return !traj->active;
}

float trajectory_get_position(const trajectory_t *traj)
{
    return (traj == 0) ? 0.0f : traj->position;
}

float trajectory_get_velocity(const trajectory_t *traj)
{
    return (traj == 0) ? 0.0f : traj->velocity;
}

float trajectory_get_acceleration(const trajectory_t *traj)
{
    return (traj == 0) ? 0.0f : traj->acceleration;
}

float trajectory_get_jerk(const trajectory_t *traj)
{
    return (traj == 0) ? 0.0f : traj->jerk;
}

float trajectory_get_progress(const trajectory_t *traj)
{
    if (traj == 0 || traj->duration <= 0.0f) {
        return 1.0f;
    }
    return scurve_clamp01(traj->elapsed / traj->duration);
}
