#include "servo_map.h"

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static uint16_t clampu16(uint16_t x, uint16_t lo, uint16_t hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void servo_cal_init(servo_cal_t *cal,
                    float min_angle_deg,
                    float max_angle_deg,
                    uint16_t min_pulse_us,
                    uint16_t max_pulse_us,
                    bool reversed)
{
    if (cal == 0) {
        return;
    }

    cal->min_angle_deg = min_angle_deg;
    cal->max_angle_deg = max_angle_deg;
    cal->min_pulse_us = min_pulse_us;
    cal->max_pulse_us = max_pulse_us;
    cal->reversed = reversed;

    if (cal->max_angle_deg <= cal->min_angle_deg) {
        cal->min_angle_deg = 0.0f;
        cal->max_angle_deg = 180.0f;
    }

    if (cal->max_pulse_us <= cal->min_pulse_us) {
        cal->min_pulse_us = 1000;
        cal->max_pulse_us = 2000;
    }
}

float servo_clamp_angle(const servo_cal_t *cal, float angle_deg)
{
    if (cal == 0) {
        return angle_deg;
    }
    return clampf(angle_deg, cal->min_angle_deg, cal->max_angle_deg);
}

uint16_t servo_angle_to_us(const servo_cal_t *cal, float angle_deg)
{
    if (cal == 0) {
        return 1500;
    }

    angle_deg = servo_clamp_angle(cal, angle_deg);

    float u = (angle_deg - cal->min_angle_deg) /
              (cal->max_angle_deg - cal->min_angle_deg);

    if (cal->reversed) {
        u = 1.0f - u;
    }

    const float pulse = (float)cal->min_pulse_us +
        u * (float)(cal->max_pulse_us - cal->min_pulse_us);

    return (uint16_t)(pulse + 0.5f);
}

float servo_us_to_angle(const servo_cal_t *cal, uint16_t pulse_us)
{
    if (cal == 0) {
        return 90.0f;
    }

    pulse_us = clampu16(pulse_us, cal->min_pulse_us, cal->max_pulse_us);

    float u = ((float)pulse_us - (float)cal->min_pulse_us) /
              (float)(cal->max_pulse_us - cal->min_pulse_us);

    if (cal->reversed) {
        u = 1.0f - u;
    }

    return cal->min_angle_deg + u * (cal->max_angle_deg - cal->min_angle_deg);
}
