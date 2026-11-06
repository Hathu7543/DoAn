#ifndef ROBOT_APP_H
#define ROBOT_APP_H
#include "main.h"

HAL_StatusTypeDef RobotApp_Init(TIM_HandleTypeDef *left_encoder,
                                TIM_HandleTypeDef *right_encoder,
                                TIM_HandleTypeDef *pwm_timer,
                                I2C_HandleTypeDef *imu_bus);
void RobotApp_Process(void);

// Existing MCUViewer / Live Expressions names are preserved.
extern volatile float target_left_rpm;
extern volatile float target_right_rpm;
extern volatile float measured_left_rpm;
extern volatile float measured_right_rpm;
extern volatile float filtered_left_rpm;
extern volatile float filtered_right_rpm;
extern volatile float pwm_left;
extern volatile float pwm_right;
extern volatile float error_left_rpm;
extern volatile float error_right_rpm;
extern volatile float integral_left_duty;
extern volatile float integral_right_duty;
extern volatile float rpm_average_left_1s;
extern volatile float rpm_average_right_1s;
#endif
