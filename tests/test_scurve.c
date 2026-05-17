#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "scurve.h"

#define EPS 1.0e-5f

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

static void test_endpoints(void)
{
    check_close("linear p(0)", scurve_eval(0.0f, SCURVE_LINEAR), 0.0f, EPS);
    check_close("linear p(1)", scurve_eval(1.0f, SCURVE_LINEAR), 1.0f, EPS);

    check_close("cubic p(0)", scurve_eval(0.0f, SCURVE_CUBIC_SMOOTHSTEP), 0.0f, EPS);
    check_close("cubic p(1)", scurve_eval(1.0f, SCURVE_CUBIC_SMOOTHSTEP), 1.0f, EPS);

    check_close("quintic p(0)", scurve_eval(0.0f, SCURVE_QUINTIC_SMOOTHERSTEP), 0.0f, EPS);
    check_close("quintic p(1)", scurve_eval(1.0f, SCURVE_QUINTIC_SMOOTHERSTEP), 1.0f, EPS);
}

static void test_midpoints(void)
{
    check_close("linear p(0.5)", scurve_eval(0.5f, SCURVE_LINEAR), 0.5f, EPS);
    check_close("cubic p(0.5)", scurve_eval(0.5f, SCURVE_CUBIC_SMOOTHSTEP), 0.5f, EPS);
    check_close("quintic p(0.5)", scurve_eval(0.5f, SCURVE_QUINTIC_SMOOTHERSTEP), 0.5f, EPS);
}

static void test_endpoint_derivatives(void)
{
    check_close("cubic zero velocity start", scurve_eval_velocity_norm(0.0f, SCURVE_CUBIC_SMOOTHSTEP), 0.0f, EPS);
    check_close("cubic zero velocity end", scurve_eval_velocity_norm(1.0f, SCURVE_CUBIC_SMOOTHSTEP), 0.0f, EPS);

    check_close("quintic zero velocity start", scurve_eval_velocity_norm(0.0f, SCURVE_QUINTIC_SMOOTHERSTEP), 0.0f, EPS);
    check_close("quintic zero velocity end", scurve_eval_velocity_norm(1.0f, SCURVE_QUINTIC_SMOOTHERSTEP), 0.0f, EPS);

    check_close("quintic zero acceleration start", scurve_eval_acceleration_norm(0.0f, SCURVE_QUINTIC_SMOOTHERSTEP), 0.0f, EPS);
    check_close("quintic zero acceleration end", scurve_eval_acceleration_norm(1.0f, SCURVE_QUINTIC_SMOOTHERSTEP), 0.0f, EPS);
}

static void test_clamping_and_monotonicity(void)
{
    check_close("clamp low", scurve_eval(-1.0f, SCURVE_QUINTIC_SMOOTHERSTEP), 0.0f, EPS);
    check_close("clamp high", scurve_eval(2.0f, SCURVE_QUINTIC_SMOOTHERSTEP), 1.0f, EPS);

    float last = scurve_eval(0.0f, SCURVE_QUINTIC_SMOOTHERSTEP);
    bool monotonic = true;
    for (int i = 1; i <= 100; ++i) {
        float s = (float)i / 100.0f;
        float p = scurve_eval(s, SCURVE_QUINTIC_SMOOTHERSTEP);
        if (p < last) {
            monotonic = false;
            break;
        }
        last = p;
    }
    check_true("quintic monotonic", monotonic);
}

int main(void)
{
    printf("Running scurve tests...\n");

    test_endpoints();
    test_midpoints();
    test_endpoint_derivatives();
    test_clamping_and_monotonicity();

    if (failures == 0) {
        printf("All scurve tests passed.\n");
        return 0;
    }

    printf("%d scurve test(s) failed.\n", failures);
    return 1;
}
