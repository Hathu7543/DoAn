#ifndef SPEED_FILTER_H
#define SPEED_FILTER_H
#include <stdint.h>
#include "motor_config.h"

typedef struct {
    float buffer[SPEED_AVERAGE_LENGTH];
    float sum;
    uint16_t index;
    uint16_t sample_count;
} SpeedFilter;

void SpeedFilter_Reset(SpeedFilter *filter);
float SpeedFilter_Update(SpeedFilter *filter, float input);
#endif
