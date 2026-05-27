#ifndef SERVO_MAP_H
#define SERVO_MAP_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float min_angle_deg;
    float max_angle_deg;
    uint16_t min_pulse_us;
    uint16_t max_pulse_us;
    bool reversed;
} servo_cal_t;

void servo_cal_init(servo_cal_t *cal,
                    float min_angle_deg,
                    float max_angle_deg,
                    uint16_t min_pulse_us,
                    uint16_t max_pulse_us,
                    bool reversed);

uint16_t servo_angle_to_us(const servo_cal_t *cal, float angle_deg);
float servo_us_to_angle(const servo_cal_t *cal, uint16_t pulse_us);
float servo_clamp_angle(const servo_cal_t *cal, float angle_deg);

#ifdef __cplusplus
}
#endif

#endif
