#include "robot_control.h"
#include "robot_math.h"
#include "imu_mpu6050.h"
#include <math.h>

volatile uint32_t robot_enable = 0;
volatile float command_v_mps = 0.1f, command_w_radps = 0.0f;
volatile float reference_v_mps, reference_w_radps;
volatile float measured_v_mps, encoder_w_radps;
// robot_fault: 0 OK, 1 IMU, 2 loop overrun, 3 invalid command/gain.
volatile uint32_t robot_fault;
volatile uint32_t yaw_feedback_enable = 1;
volatile float yaw_kp = 0.3f, yaw_correction_radps;
volatile float accel_reference_mps2, alpha_reference_radps2;
volatile float force_reference_n;
// Unknown inertia: zero means not identified, NOT a zero-inertia vehicle.
volatile float body_inertia_kgm2 = 0.0f, yaw_torque_reference_nm;
volatile float wheel_torque_left_nm, wheel_torque_right_nm;
volatile uint32_t dynamics_valid;


void Robot_Init(I2C_HandleTypeDef *bus)
{
    Imu_Init(bus);
}

void Robot_Update(uint32_t elapsed_ms, float left_rpm, float right_rpm,
                  int encoder_moving, float *left_target, float *right_target)
{
    float v, w;
    RobotForward(left_rpm, right_rpm, &v, &w);
    measured_v_mps = v;
    encoder_w_radps = w;
    Imu_Update(encoder_moving);
    if (elapsed_ms > 50U) robot_fault = 2;
    float requested_v = command_v_mps, requested_w = command_w_radps;
    float kp = yaw_kp;
    if (!isfinite(requested_v) || !isfinite(requested_w) || !isfinite(kp) || kp < 0.0f) {
        robot_fault = 3;
    }
    if (imu_state != 2 || robot_fault) robot_enable = 0;
    *left_target = *right_target = 0.0f;
    accel_reference_mps2 = alpha_reference_radps2 = 0.0f;
    force_reference_n = yaw_torque_reference_nm = 0.0f;
    wheel_torque_left_nm = wheel_torque_right_nm = 0.0f;
    dynamics_valid = 0;
    if (!robot_enable || (requested_v == 0.0f && requested_w == 0.0f)) {
        reference_v_mps = reference_w_radps = yaw_correction_radps = 0.0f;
        return; // Stop bypasses ramp and yaw correction: no unexpected spin.
    }
    float dt = (float)elapsed_ms * 0.001f;
    float old_v = reference_v_mps, old_w = reference_w_radps;
    reference_v_mps = RobotSlew(old_v, RobotClamp(requested_v, -ROBOT_MAX_V, ROBOT_MAX_V),
                                ROBOT_ACCEL_MPS2 * dt);
    reference_w_radps = RobotSlew(old_w, RobotClamp(requested_w, -ROBOT_MAX_W, ROBOT_MAX_W),
                                  ROBOT_ACCEL_RADPS2 * dt);
    accel_reference_mps2 = (reference_v_mps - old_v) / dt;
    alpha_reference_radps2 = (reference_w_radps - old_w) / dt;
    // Low-gain bounded outer yaw-rate feedback; wheel PID remains inner loop.
    yaw_correction_radps = yaw_feedback_enable ? RobotClamp(
        RobotClamp(kp, 0.0f, 1.0f) * (reference_w_radps - gyro_w_radps), -0.15f, 0.15f) : 0.0f;
    RobotInverse(reference_v_mps, reference_w_radps + yaw_correction_radps,
                  left_target, right_target);
    // Ideal planar model, diagnostic only: no uncalibrated torque-to-duty mapping.
    force_reference_n = ROBOT_MASS_KG * accel_reference_mps2;
    float inertia = body_inertia_kgm2;
    if (isfinite(inertia) && inertia > 0.0f) {
        yaw_torque_reference_nm = inertia * alpha_reference_radps2;
        wheel_torque_left_nm = ROBOT_RADIUS_M * (0.5f * force_reference_n -
                                  yaw_torque_reference_nm / ROBOT_TRACK_M);
        wheel_torque_right_nm = ROBOT_RADIUS_M * (0.5f * force_reference_n +
                                   yaw_torque_reference_nm / ROBOT_TRACK_M);
        dynamics_valid = 1;
    }
}
