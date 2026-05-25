#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "scurve.h"

typedef enum {
    TRAJ_STATUS_OK = 0,
    TRAJ_STATUS_NULL_POINTER,
    TRAJ_STATUS_INVALID_DURATION,
    TRAJ_STATUS_INVALID_LIMIT
} traj_status_t;

typedef struct {
    float start;
    float target;
    float duration;
    float elapsed;

    float position;
    float velocity;
    float acceleration;
    float jerk;

    scurve_mode_t mode;
    bool active;
} trajectory_t;

/**
 * Initialize a fixed-duration trajectory.
 *
 * Units are arbitrary but must be consistent. For a servo, position may be
 * degrees or radians. Duration and dt are seconds.
 */
traj_status_t trajectory_init(
    trajectory_t *traj,
    float start,
    float target,
    float duration,
    scurve_mode_t mode
);

/**
 * Initialize a trajectory and automatically choose duration based on limits.
 *
 * vmax has units position/s, amax position/s^2, jmax position/s^3.
 * Set any limit <= 0 to ignore that limit.
 */
traj_status_t trajectory_init_limited(
    trajectory_t *traj,
    float start,
    float target,
    float requested_duration,
    float vmax,
    float amax,
    float jmax,
    scurve_mode_t mode
);

/** Compute the minimum duration required to satisfy selected limits. */
float trajectory_min_duration(
    float delta,
    float vmax,
    float amax,
    float jmax,
    scurve_mode_t mode
);

/** Advance trajectory by dt seconds and return current position. */
float trajectory_update(trajectory_t *traj, float dt);

/** Reset elapsed time and output back to start. */
void trajectory_reset(trajectory_t *traj);

/** Retarget from current position to a new target. */
traj_status_t trajectory_retarget(
    trajectory_t *traj,
    float new_target,
    float duration,
    scurve_mode_t mode
);

bool trajectory_is_done(const trajectory_t *traj);
float trajectory_get_position(const trajectory_t *traj);
float trajectory_get_velocity(const trajectory_t *traj);
float trajectory_get_acceleration(const trajectory_t *traj);
float trajectory_get_jerk(const trajectory_t *traj);
float trajectory_get_progress(const trajectory_t *traj);

#ifdef __cplusplus
}
#endif

#endif /* TRAJECTORY_H */
