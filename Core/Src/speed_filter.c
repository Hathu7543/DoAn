#include "speed_filter.h"

void SpeedFilter_Reset(SpeedFilter *filter)
{
    for (uint16_t i = 0; i < SPEED_AVERAGE_LENGTH; ++i) {
        filter->buffer[i] = 0.0f;
    }
    filter->sum = 0.0f;
    filter->index = 0;
    filter->sample_count = 0;
}

float SpeedFilter_Update(SpeedFilter *filter, float input)
{
    filter->sum -= filter->buffer[filter->index];
    filter->buffer[filter->index] = input;
    filter->sum += input;
    if (++filter->index >= SPEED_AVERAGE_LENGTH) {
        filter->index = 0;
    }
    if (filter->sample_count < SPEED_AVERAGE_LENGTH) {
        ++filter->sample_count;
    }
    return filter->sum / (float)filter->sample_count;
}
