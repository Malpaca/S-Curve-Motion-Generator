#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "trajectory.h"

#define EPS 1.0e-4f

static int failures = 0;

static void check_close(const char *name, float actual, float expected, float eps)
{
    if (fabsf(actual - expected) > eps) {
        printf("FAIL: %s: actual=%f expected=%f\n", name, actual, expected);
        failures++;
    } else {
        printf("PASS: %s\n", name);
    }
}

static void check_true(const char *name, bool condition)
{
    if (!condition) {
        printf("FAIL: %s\n", name);
        failures++;
    } else {
        printf("PASS: %s\n", name);
    }
}

static void test_fixed_duration(void)
{
    trajectory_t traj;
    traj_status_t status = trajectory_init(&traj, 30.0f, 90.0f, 1.0f, SCURVE_QUINTIC_SMOOTHERSTEP);
    check_true("trajectory init ok", status == TRAJ_STATUS_OK);
    check_close("initial position", trajectory_get_position(&traj), 30.0f, EPS);

    trajectory_update(&traj, 0.5f);
    check_close("halfway position", trajectory_get_position(&traj), 60.0f, EPS);

    trajectory_update(&traj, 0.5f);
    check_close("final position", trajectory_get_position(&traj), 90.0f, EPS);
    check_true("trajectory done", trajectory_is_done(&traj));
    check_close("final velocity zero", trajectory_get_velocity(&traj), 0.0f, EPS);
    check_close("final acceleration zero", trajectory_get_acceleration(&traj), 0.0f, EPS);
}

static void test_negative_direction(void)
{
    trajectory_t traj;
    trajectory_init(&traj, 100.0f, 40.0f, 2.0f, SCURVE_CUBIC_SMOOTHSTEP);

    trajectory_update(&traj, 1.0f);
    check_close("negative direction halfway", trajectory_get_position(&traj), 70.0f, EPS);

    trajectory_update(&traj, 1.0f);
    check_close("negative direction final", trajectory_get_position(&traj), 40.0f, EPS);
    check_true("negative direction done", trajectory_is_done(&traj));
}

static void test_min_duration(void)
{
    /* 90 degrees move, quintic, acceleration limit dominates here. */
    const float delta = 90.0f;
    const float vmax = 0.0f;       /* ignored */
    const float amax = 180.0f;     /* deg/s^2 */
    const float jmax = 0.0f;       /* ignored */

    float tmin = trajectory_min_duration(delta, vmax, amax, jmax, SCURVE_QUINTIC_SMOOTHERSTEP);
    float expected = sqrtf(delta * 5.773503f / amax);
    check_close("min duration acceleration limit", tmin, expected, 1.0e-3f);
}

static void test_limited_init(void)
{
    trajectory_t traj;

    traj_status_t status = trajectory_init_limited(
        &traj,
        0.0f,
        90.0f,
        0.1f,       /* requested duration too short */
        0.0f,       /* ignore velocity */
        180.0f,     /* deg/s^2 */
        0.0f,       /* ignore jerk */
        SCURVE_QUINTIC_SMOOTHERSTEP
    );

    check_true("limited init ok", status == TRAJ_STATUS_OK);
    check_true("duration increased by limit", traj.duration > 0.1f);
}

static void test_retarget(void)
{
    trajectory_t traj;
    trajectory_init(&traj, 0.0f, 100.0f, 1.0f, SCURVE_LINEAR);
    trajectory_update(&traj, 0.25f);
    check_close("position before retarget", traj.position, 25.0f, EPS);

    trajectory_retarget(&traj, 75.0f, 1.0f, SCURVE_LINEAR);
    check_close("retarget start equals current", traj.start, 25.0f, EPS);
    trajectory_update(&traj, 1.0f);
    check_close("retarget final", traj.position, 75.0f, EPS);
}

int main(void)
{
    printf("Running trajectory tests...\n");

    test_fixed_duration();
    test_negative_direction();
    test_min_duration();
    test_limited_init();
    test_retarget();

    if (failures == 0) {
        printf("All trajectory tests passed.\n");
        return 0;
    }

    printf("%d trajectory test(s) failed.\n", failures);
    return 1;
}
