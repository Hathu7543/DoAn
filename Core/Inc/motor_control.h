#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include "main.h"
#include "motor_config.h"
#include "speed_filter.h"

typedef struct
{
    TIM_HandleTypeDef *encoder_timer;
    uint32_t pwm_channel;
    GPIO_TypeDef *direction_port;
    uint16_t in1_pin;
    uint16_t in2_pin;
    int8_t encoder_direction;
    int8_t drive_direction;
    TIM_HandleTypeDef *pwm_timer;
    uint16_t previous_count;
    int64_t total_counts;
    SpeedFilter speed_filter;
    float previous_speed;
    float integral_output;
    int32_t report_counts;
    uint32_t report_elapsed_ms;
    int8_t applied_direction;
    int8_t target_direction;
} MotorController;

extern volatile float Kp, KpFine, Ki, Kd;

void MotorController_Reset(MotorController *motor);

void Motor_SetDuty(MotorController *motor,
                          float logical_duty,
                          volatile float *pwm_value);

void Motor_Update(MotorController *motor,
                         uint16_t count_now,
                         uint32_t elapsed_ms,
                         float setpoint,
                         volatile float *measured_rpm,
                         volatile float *filtered_rpm,
                         volatile float *error_rpm,
                         volatile float *integral_duty,
                         volatile float *pwm_value,
                         volatile float *rpm_average);
#endif
