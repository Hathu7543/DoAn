#include "motor_control.h"
#include <math.h>

// Gains expressed in percent duty, rather than raw timer counts.
// Reduce proportional sensitivity to quantized speed feedback and MA delay.
// These are trial gains; validate settling and load response on the motor.
volatile float Kp = 0.6f;
volatile float KpFine = 0.15f; // P slope inside +/-2 RPM; Ki still sees full error.
volatile float Ki = 1.0f;
volatile float Kd = 0.0f;

void MotorController_Reset(MotorController *motor)
{
    SpeedFilter_Reset(&motor->speed_filter);
    motor->previous_speed = 0.0f;
    motor->integral_output = 0.0f;
    motor->report_counts = 0;
    motor->report_elapsed_ms = 0;
    motor->applied_direction = 0;
    motor->target_direction = 0;
    __HAL_TIM_SET_COUNTER(motor->encoder_timer, 0);
    motor->previous_count = 0;
    motor->total_counts = 0;
}

static float Clamp(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static float Proportional_Update(float error, float kp, float kp_fine)
{
    // Continuous piecewise-linear P: no dead band, no switch in output.
    // Small encoder steps cause smaller duty changes near the setpoint.
    kp = fmaxf(kp, 0.0f);
    kp_fine = Clamp(kp_fine, 0.0f, kp);
    float magnitude = fabsf(error);
    float fine_error = fminf(magnitude, P_FINE_BAND_RPM);
    float output = kp_fine * fine_error + kp * (magnitude - fine_error);
    return (error < 0.0f) ? -output : output;
}

static void SpeedReport_Update(MotorController *motor,
                               int32_t counts,
                               uint32_t elapsed_ms,
                               volatile float *rpm_average)
{
    motor->report_counts += counts;
    motor->report_elapsed_ms += elapsed_ms;
    if (motor->report_elapsed_ms >= SPEED_REPORT_PERIOD_MS) {
        *rpm_average = (float)motor->report_counts * 60000.0f /
                       (ENCODER_COUNTS_PER_REV * (float)motor->report_elapsed_ms);
        motor->report_counts = 0;
        motor->report_elapsed_ms = 0;
    }
}

void Motor_SetDuty(MotorController *motor,
                          float logical_duty,
                          volatile float *pwm_value)
{
    logical_duty = Clamp(logical_duty, -100.0f, 100.0f);
    float physical_duty = logical_duty * (float)motor->drive_direction;
    float magnitude = fabsf(physical_duty);
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(motor->pwm_timer) + 1U;
    uint32_t compare = (uint32_t)(magnitude * (float)period / 100.0f + 0.5f);
    int8_t desired_direction = (physical_duty > 0.0f) ? 1 :
                               ((physical_duty < 0.0f) ? -1 : 0);

    // Hold IN1/IN2 low for a complete control cycle before reversing.
    if (desired_direction != 0 && motor->applied_direction != 0 &&
        desired_direction != motor->applied_direction) {
        __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, 0);
        HAL_GPIO_WritePin(motor->direction_port,
                          motor->in1_pin | motor->in2_pin, GPIO_PIN_RESET);
        motor->applied_direction = 0;
        *pwm_value = 0.0f;
        return;
    }

    if (compare == 0U) {
        HAL_GPIO_WritePin(motor->direction_port,
                          motor->in1_pin | motor->in2_pin, GPIO_PIN_RESET);
    } else if (physical_duty > 0.0f) {
        HAL_GPIO_WritePin(motor->direction_port, motor->in2_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->direction_port, motor->in1_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(motor->direction_port, motor->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->direction_port, motor->in2_pin, GPIO_PIN_SET);
    }
    __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, compare);
    motor->applied_direction = desired_direction;
    *pwm_value = (logical_duty < 0.0f ? -1.0f : 1.0f) *
                 (100.0f * (float)compare / (float)period);
}

void Motor_Update(MotorController *motor,
                         uint16_t count_now,
                         uint32_t elapsed_ms,
                         float setpoint,
                         volatile float *measured_rpm,
                         volatile float *filtered_rpm,
                         volatile float *error_rpm,
                         volatile float *integral_duty,
                         volatile float *pwm_value,
                         volatile float *rpm_average)
{
    float dt = (float)elapsed_ms * 0.001f;
    int32_t delta_count = (int32_t)count_now - (int32_t)motor->previous_count;
    if (delta_count > 32767) delta_count -= 65536;
    if (delta_count < -32768) delta_count += 65536;
    motor->previous_count = count_now;

    int32_t directed_counts = motor->encoder_direction * delta_count;
    motor->total_counts += directed_counts;
    SpeedReport_Update(motor, directed_counts, elapsed_ms, rpm_average);
    float speed = (float)directed_counts * 60.0f /
                  (ENCODER_COUNTS_PER_REV * dt);
    *measured_rpm = speed;
    float filtered = SpeedFilter_Update(&motor->speed_filter, speed);
    float derivative = -(filtered - motor->previous_speed) / dt;
    motor->previous_speed = filtered;
    *filtered_rpm = filtered;

    if (!isfinite(setpoint) || fabsf(setpoint) <= TARGET_STOP_EPSILON_RPM) {
        motor->integral_output = 0.0f;
        *integral_duty = 0.0f;
        *error_rpm = 0.0f;
        motor->target_direction = 0;
        Motor_SetDuty(motor, 0.0f, pwm_value);
        return;
    }

    int8_t requested_direction = (setpoint > 0.0f) ? 1 : -1;
    if (motor->target_direction != 0 &&
        requested_direction != motor->target_direction) {
        motor->integral_output = 0.0f;
    }
    motor->target_direction = requested_direction;

    float error = setpoint - filtered;
    *error_rpm = error;
    float kp = Kp;
    float kp_fine = KpFine;
    float ki = Ki;
    float kd = Kd;
    float output_without_i = Proportional_Update(error, kp, kp_fine) +
                             kd * derivative;
    if (ki > 0.0f) {
        float change = ki * error * dt;
        float candidate = Clamp(motor->integral_output + change, -100.0f, 100.0f);
        float candidate_output = output_without_i + candidate;
        if ((candidate_output >= -100.0f && candidate_output <= 100.0f) ||
            (candidate_output > 100.0f && change < 0.0f) ||
            (candidate_output < -100.0f && change > 0.0f)) {
            motor->integral_output = candidate;
        }
    } else {
        motor->integral_output = 0.0f;
    }
    *integral_duty = motor->integral_output;
    Motor_SetDuty(motor, output_without_i + motor->integral_output, pwm_value);
}
