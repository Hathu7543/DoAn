#ifndef IMU_MPU6050_H
#define IMU_MPU6050_H
#include "main.h"

// Call once after I2C init, then once per control cycle.
void Imu_Init(I2C_HandleTypeDef *bus);
void Imu_Update(int encoder_moving);

// 0 init, 1 calibrating, 2 ready, 3 fault (reset required).
extern volatile uint32_t imu_state, imu_calibration_samples, imu_error_count;
extern volatile float accel_x_mps2, accel_y_mps2, accel_z_mps2;
extern volatile float accel_uncal_x_mps2, accel_uncal_y_mps2, accel_uncal_z_mps2;
extern volatile float gyro_bias_z_dps, gyro_w_radps, yaw_relative_rad;
#endif
