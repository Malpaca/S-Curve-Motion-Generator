#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <stdbool.h>
#include "scurve.h"

#ifdef __cplusplus
extern "C" {
#endif

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

void trajectory_init(trajectory_t *traj,
                     float start,
                     float target,
                     float duration,
                     scurve_mode_t mode);

void trajectory_init_limited(trajectory_t *traj,
                             float start,
                             float target,
                             float requested_duration,
                             float max_velocity,
                             float max_acceleration,
                             float max_jerk,
                             scurve_mode_t mode);

float trajectory_update(trajectory_t *traj, float dt);
void trajectory_sample(trajectory_t *traj, float elapsed_time);
void trajectory_retarget_from_current(trajectory_t *traj,
                                      float new_target,
                                      float duration,
                                      scurve_mode_t mode);

float trajectory_min_duration(float delta,
                              float max_velocity,
                              float max_acceleration,
                              float max_jerk,
                              scurve_mode_t mode);

bool trajectory_is_done(const trajectory_t *traj);

#ifdef __cplusplus
}
#endif

#endif
