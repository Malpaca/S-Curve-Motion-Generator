#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "sequence.h"

int main(void)
{
    const sequence_point_t triangle[] = {
        { .value = {60.0f, 80.0f, 0.0f}, .duration_to_next_s = 0.5f },
        { .value = {120.0f, 80.0f, 0.0f}, .duration_to_next_s = 0.5f },
        { .value = {90.0f, 45.0f, 0.0f}, .duration_to_next_s = 0.5f }
    };

    sequence_t seq;
    assert(sequence_init(&seq, triangle, 3, 2, SCURVE_QUINTIC_SMOOTHERSTEP, true));
    assert(sequence_is_active(&seq));

    const float hip0 = sequence_get_axis(&seq, 0);
    const float knee0 = sequence_get_axis(&seq, 1);
    assert(fabsf(hip0 - 60.0f) < 1e-4f);
    assert(fabsf(knee0 - 80.0f) < 1e-4f);

    for (int i = 0; i < 30; ++i) {
        sequence_update(&seq, 0.02f);
    }

    assert(sequence_is_active(&seq));
    printf("test_sequence: PASS\n");
    return 0;
}
