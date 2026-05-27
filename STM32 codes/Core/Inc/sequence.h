#ifndef SEQUENCE_H
#define SEQUENCE_H

#include <stdbool.h>
#include <stddef.h>
#include "trajectory.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum number of axes that one sequence point can control, e.g. hip/knee/ankle. */
#ifndef SEQUENCE_MAX_AXES
#define SEQUENCE_MAX_AXES 3
#endif

/* Maximum number of keyframe points stored inside a sequence object. */
#ifndef SEQUENCE_MAX_POINTS
#define SEQUENCE_MAX_POINTS 8
#endif

/* One keyframe in a multi-axis motion sequence. */
typedef struct {
    /* Target value for each axis at this keyframe, usually servo angles in degrees. */
    float value[SEQUENCE_MAX_AXES];

    /* Time in seconds to move from this point to the next point. */
    float duration_to_next_s;
} sequence_point_t;

/* Runtime state for a looping or one-shot multi-axis trajectory sequence. */
typedef struct {
    /* Local copy of the user-supplied keyframe points. */
    sequence_point_t points[SEQUENCE_MAX_POINTS];

    /* One trajectory object per controlled axis. */
    trajectory_t axis[SEQUENCE_MAX_AXES];

    /* Number of valid keyframe points stored in points[]. */
    size_t point_count;

    /* Number of active axes used from each point's value[] array. */
    size_t axis_count;

    /* Index of the current start point for the active segment. */
    size_t current_point;

    /* S-curve mode used when moving between keyframes. */
    scurve_mode_t mode;

    /* True if the sequence should restart from the first point after the last segment. */
    bool loop;

    /* True while the sequence is running and update calls should advance it. */
    bool active;
} sequence_t;

/* Initialise a multi-axis keyframe sequence from an array of points. */
bool sequence_init(sequence_t *seq,
                   const sequence_point_t *points,
                   size_t point_count,
                   size_t axis_count,
                   scurve_mode_t mode,
                   bool loop);

/* Start or resume the sequence from its current point. */
void sequence_start(sequence_t *seq);

/* Stop the sequence without clearing its current output values. */
void sequence_stop(sequence_t *seq);

/* Advance all active axes by dt seconds and start the next segment when needed. */
void sequence_update(sequence_t *seq, float dt);

/* Change the active segment's S-curve mode immediately while preserving the current position. */
void sequence_set_mode(sequence_t *seq, scurve_mode_t mode);

/* Return the current output value for one axis, or 0.0f if the index is invalid. */
float sequence_get_axis(const sequence_t *seq, size_t axis_index);

/* Return true if the sequence is currently active. */
bool sequence_is_active(const sequence_t *seq);

/* Return the index of the current keyframe segment start point. */
size_t sequence_get_current_point(const sequence_t *seq);

#ifdef __cplusplus
}
#endif

#endif
