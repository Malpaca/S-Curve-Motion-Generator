#include "sequence.h"

static size_t next_index(const sequence_t *seq, size_t i)
{
    if ((i + 1u) < seq->point_count) {
        return i + 1u;
    }
    return seq->loop ? 0u : i;
}

static void start_segment(sequence_t *seq, size_t from_index)
{
    if (seq == 0 || seq->point_count < 2u) {
        return;
    }

    const size_t to_index = next_index(seq, from_index);
    const float duration = seq->points[from_index].duration_to_next_s;

    for (size_t axis = 0u; axis < seq->axis_count; ++axis) {
        trajectory_init(&seq->axis[axis],
                        seq->points[from_index].value[axis],
                        seq->points[to_index].value[axis],
                        duration,
                        seq->mode);
    }

    seq->current_point = from_index;
    seq->active = true;
}

bool sequence_init(sequence_t *seq,
                   const sequence_point_t *points,
                   size_t point_count,
                   size_t axis_count,
                   scurve_mode_t mode,
                   bool loop)
{
    if (seq == 0 || points == 0) {
        return false;
    }
    if (point_count < 2u || point_count > SEQUENCE_MAX_POINTS) {
        return false;
    }
    if (axis_count < 1u || axis_count > SEQUENCE_MAX_AXES) {
        return false;
    }

    seq->point_count = point_count;
    seq->axis_count = axis_count;
    seq->current_point = 0u;
    seq->mode = mode;
    seq->loop = loop;
    seq->active = false;

    for (size_t i = 0u; i < point_count; ++i) {
        seq->points[i] = points[i];
    }

    sequence_start(seq);
    return true;
}

void sequence_start(sequence_t *seq)
{
    if (seq == 0) {
        return;
    }
    start_segment(seq, 0u);
}

void sequence_stop(sequence_t *seq)
{
    if (seq == 0) {
        return;
    }
    seq->active = false;
}

void sequence_update(sequence_t *seq, float dt)
{
    if (seq == 0 || !seq->active) {
        return;
    }

    bool all_done = true;
    for (size_t axis = 0u; axis < seq->axis_count; ++axis) {
        trajectory_update(&seq->axis[axis], dt);
        if (!trajectory_is_done(&seq->axis[axis])) {
            all_done = false;
        }
    }

    if (!all_done) {
        return;
    }

    const size_t ni = next_index(seq, seq->current_point);

    if (!seq->loop && ni == seq->current_point) {
        seq->active = false;
        return;
    }

    start_segment(seq, ni);
}

float sequence_get_axis(const sequence_t *seq, size_t axis_index)
{
    if (seq == 0 || axis_index >= seq->axis_count) {
        return 0.0f;
    }
    return seq->axis[axis_index].position;
}

bool sequence_is_active(const sequence_t *seq)
{
    return (seq != 0) && seq->active;
}

size_t sequence_get_current_point(const sequence_t *seq)
{
    if (seq == 0) {
        return 0u;
    }
    return seq->current_point;
}
