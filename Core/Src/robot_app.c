#include "robot_app.h"
#include "motor_control.h"
#include "robot_control.h"
#include "telemetry_uart.h"

volatile float target_left_rpm = 0.0f;
volatile float target_right_rpm = 0.0f;
volatile float measured_left_rpm = 0.0f;
volatile float measured_right_rpm = 0.0f;
volatile float filtered_left_rpm = 0.0f;
volatile float filtered_right_rpm = 0.0f;
volatile float pwm_left = 0.0f;   // Signed duty: positive means vehicle-forward.
volatile float pwm_right = 0.0f;
volatile float error_left_rpm = 0.0f;
volatile float error_right_rpm = 0.0f;
volatile float integral_left_duty = 0.0f;
volatile float integral_right_duty = 0.0f;
volatile float rpm_average_left_1s = 0.0f;
volatile float rpm_average_right_1s = 0.0f;
static MotorController left_motor;
static MotorController right_motor;
static uint32_t last_control_tick = 0;

HAL_StatusTypeDef RobotApp_Init(TIM_HandleTypeDef *left_encoder,
                                TIM_HandleTypeDef *right_encoder,
                                TIM_HandleTypeDef *pwm_timer,
                                I2C_HandleTypeDef *imu_bus)
{
  Telemetry_Init();

  left_motor.pwm_timer = pwm_timer;
  right_motor.pwm_timer = pwm_timer;

  left_motor.encoder_timer = left_encoder;
  left_motor.pwm_channel = TIM_CHANNEL_1;
  left_motor.direction_port = GPIOA;
  left_motor.in1_pin = GPIO_PIN_1;
  left_motor.in2_pin = GPIO_PIN_2;
  left_motor.encoder_direction = LEFT_ENCODER_DIRECTION;
  left_motor.drive_direction = LEFT_DRIVE_DIRECTION;

  right_motor.encoder_timer = right_encoder;
  right_motor.pwm_channel = TIM_CHANNEL_2;
  right_motor.direction_port = GPIOB;
  right_motor.in1_pin = GPIO_PIN_0;
  right_motor.in2_pin = GPIO_PIN_1;
  right_motor.encoder_direction = RIGHT_ENCODER_DIRECTION;
  right_motor.drive_direction = RIGHT_DRIVE_DIRECTION;

  MotorController_Reset(&left_motor);
  MotorController_Reset(&right_motor);
  __HAL_TIM_SET_COMPARE(pwm_timer, TIM_CHANNEL_2, 0);
  Motor_SetDuty(&left_motor, 0.0f, &pwm_left);
  Motor_SetDuty(&right_motor, 0.0f, &pwm_right);
  Robot_Init(imu_bus);

  if (HAL_TIM_Encoder_Start(left_encoder, TIM_CHANNEL_ALL) != HAL_OK ||
      HAL_TIM_Encoder_Start(right_encoder, TIM_CHANNEL_ALL) != HAL_OK) {
      return HAL_ERROR;
  }
  if (HAL_TIM_PWM_Start(pwm_timer, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_Start(pwm_timer, TIM_CHANNEL_2) != HAL_OK) {
      return HAL_ERROR;
  }
  last_control_tick = HAL_GetTick();
  return HAL_OK;
}

void RobotApp_Process(void)
{
    Telemetry_Service();
    uint32_t now = HAL_GetTick();
    uint32_t elapsed_ms = now - last_control_tick;
    if (elapsed_ms >= CONTROL_PERIOD_MS) {
        last_control_tick = now;
        // Capture both encoder counters before I2C access for a common sample time.
        uint16_t left_count = (uint16_t)__HAL_TIM_GET_COUNTER(left_motor.encoder_timer);
        uint16_t right_count = (uint16_t)__HAL_TIM_GET_COUNTER(right_motor.encoder_timer);
        int encoder_moving =
            left_count != left_motor.previous_count ||
            right_count != right_motor.previous_count;
        float left_target, right_target;
        Robot_Update(elapsed_ms, filtered_left_rpm, filtered_right_rpm,
                     encoder_moving, &left_target, &right_target);
        target_left_rpm = left_target;
        target_right_rpm = right_target;
        Motor_Update(&left_motor, left_count, elapsed_ms, target_left_rpm,
                     &measured_left_rpm, &filtered_left_rpm,
                     &error_left_rpm, &integral_left_duty,
                     &pwm_left, &rpm_average_left_1s);
        Motor_Update(&right_motor, right_count, elapsed_ms, target_right_rpm,
                     &measured_right_rpm, &filtered_right_rpm,
                     &error_right_rpm, &integral_right_duty,
                     &pwm_right, &rpm_average_right_1s);
        Telemetry_Send(now, left_motor.total_counts, right_motor.total_counts,
                       gyro_w_radps, accel_x_mps2, accel_y_mps2, accel_z_mps2,
                       imu_state, robot_fault, robot_enable);
    }
}
