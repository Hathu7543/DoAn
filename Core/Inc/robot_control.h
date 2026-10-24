#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H
#include "main.h"

// Watch/write these in CubeIDE Live Expressions or MCUViewer.
extern volatile uint32_t robot_enable;
extern volatile float command_v_mps, command_w_radps;
extern volatile float reference_v_mps, reference_w_radps;
extern volatile float measured_v_mps, encoder_w_radps, gyro_w_radps;
extern volatile float accel_x_mps2, accel_y_mps2, accel_z_mps2;
extern volatile float accel_uncal_x_mps2, accel_uncal_y_mps2, accel_uncal_z_mps2;
extern volatile float gyro_bias_z_dps, yaw_relative_rad;
extern volatile uint32_t imu_state, imu_calibration_samples, imu_error_count;
extern volatile uint32_t robot_fault;
extern volatile uint32_t yaw_feedback_enable;
extern volatile float yaw_kp, yaw_correction_radps;
extern volatile float accel_reference_mps2, alpha_reference_radps2;
extern volatile float force_reference_n;
extern volatile float body_inertia_kgm2, yaw_torque_reference_nm;
extern volatile float wheel_torque_left_nm, wheel_torque_right_nm;
extern volatile uint32_t dynamics_valid;

void Robot_Init(I2C_HandleTypeDef *bus);
void Robot_Update(uint32_t elapsed_ms, float left_rpm, float right_rpm,
                  int encoder_moving, float *left_target, float *right_target);
#endif
