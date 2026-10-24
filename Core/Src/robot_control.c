#include "robot_control.h"
#include "robot_math.h"
#include <math.h>

// Mount sensor flat: +X forward, +Y left, +Z up. No axis remapping assumed.
// Datasheet: RM-MPU-6000A-00, gyro +/-250 deg/s, accel +/-2 g.
#define MPU_ADDRESS (0x68U << 1) // AD0 must connect to GND.
#define I2C_TIMEOUT_MS 3U
#define CALIBRATION_SAMPLES 300U
#define DEG_TO_RAD (ROBOT_PI / 180.0f)
#define GRAVITY_MPS2 9.80665f
// User's six-position readings in m/s^2, before acceleration correction:
// X: +9.8/-9.5, Y: +9.9/-9.7, Z: +11.6/-8.0.
// Apply exactly once. These coefficients belong to this particular module.
#define ACCEL_BIAS_X_MPS2 0.15f
#define ACCEL_BIAS_Y_MPS2 0.10f
#define ACCEL_BIAS_Z_MPS2 1.80f
#define ACCEL_SCALE_X (2.0f * GRAVITY_MPS2 / 19.3f)
#define ACCEL_SCALE_Y (2.0f * GRAVITY_MPS2 / 19.6f)
#define ACCEL_SCALE_Z (2.0f * GRAVITY_MPS2 / 19.6f)

volatile uint32_t robot_enable = 0;
volatile float command_v_mps = 0.1f, command_w_radps = 0.0f;
volatile float reference_v_mps, reference_w_radps;
volatile float measured_v_mps, encoder_w_radps, gyro_w_radps;
volatile float accel_x_mps2, accel_y_mps2, accel_z_mps2;
volatile float accel_uncal_x_mps2, accel_uncal_y_mps2, accel_uncal_z_mps2;
volatile float gyro_bias_z_dps, yaw_relative_rad;
// imu_state: 0 init, 1 calibrating, 2 ready, 3 fault. Faults require reset.
volatile uint32_t imu_state, imu_calibration_samples, imu_error_count;
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

static I2C_HandleTypeDef *imu_bus;
static float bias_sum, bias_square_sum;
static uint32_t last_sample_tick;
static int gyro_filter_initialized;

static int WriteRegister(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(imu_bus, MPU_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT,
                             &value, 1, I2C_TIMEOUT_MS) == HAL_OK;
}

static int16_t Signed16(const uint8_t *p)
{
    int32_t value = ((uint32_t)p[0] << 8) | p[1];
    return (int16_t)(value >= 32768 ? value - 65536 : value);
}

static void ImuFault(void)
{
    ++imu_error_count;
    imu_state = 3;
    robot_fault = 1;
    robot_enable = 0;
}

void Robot_Init(I2C_HandleTypeDef *bus)
{
    imu_bus = bus;
    robot_enable = 0;
    imu_state = 0;
    uint8_t identity = 0;
    if (HAL_I2C_Mem_Read(bus, MPU_ADDRESS, 0x75, I2C_MEMADD_SIZE_8BIT,
                         &identity, 1, I2C_TIMEOUT_MS) != HAL_OK || identity != 0x68) {
        ImuFault();
        return;
    }
    if (!WriteRegister(0x6B, 0x80)) { ImuFault(); return; }
    HAL_Delay(100); // Only at boot, with motor outputs disabled.
    if (!WriteRegister(0x6B, 0x01) || // Wake, PLL X gyro clock.
        !WriteRegister(0x6C, 0x00) || // All axes enabled.
        !WriteRegister(0x1A, 0x03) || // DLPF: gyro 42 Hz, accel 44 Hz.
        !WriteRegister(0x19, 0x09) || // 1 kHz / (9+1) = 100 Hz.
        !WriteRegister(0x1B, 0x00) || // +/-250 deg/s: 131 LSB/(deg/s).
        !WriteRegister(0x1C, 0x00) || // +/-2 g: 16384 LSB/g.
        !WriteRegister(0x38, 0x01)) { // Data-ready status, INT wire not used.
        ImuFault();
        return;
    }
    HAL_Delay(100);
    last_sample_tick = HAL_GetTick();
    imu_state = 1;
}

static void ResetCalibration(void)
{
    imu_calibration_samples = 0;
    bias_sum = bias_square_sum = 0.0f;
}

static void ReadImu(int encoder_moving)
{
    if (imu_state == 3) return;
    // INT_STATUS + coherent 14-byte acceleration/temperature/gyro sample.
    uint8_t data[15];
    if (HAL_I2C_Mem_Read(imu_bus, MPU_ADDRESS, 0x3A, I2C_MEMADD_SIZE_8BIT,
                         data, sizeof(data), I2C_TIMEOUT_MS) != HAL_OK) {
        ImuFault();
        return;
    }
    uint32_t now = HAL_GetTick();
    if (!(data[0] & 1U)) {
        if ((uint32_t)(now - last_sample_tick) >= 100U) ImuFault();
        return;
    }
    float sample_dt = (float)(uint32_t)(now - last_sample_tick) * 0.001f;
    last_sample_tick = now;
    float raw_ax = (float)Signed16(&data[1]) * (GRAVITY_MPS2 / 16384.0f);
    float raw_ay = (float)Signed16(&data[3]) * (GRAVITY_MPS2 / 16384.0f);
    float raw_az = (float)Signed16(&data[5]) * (GRAVITY_MPS2 / 16384.0f);
    accel_uncal_x_mps2 = raw_ax;
    accel_uncal_y_mps2 = raw_ay;
    accel_uncal_z_mps2 = raw_az;
    float corrected_ax = (raw_ax - ACCEL_BIAS_X_MPS2) * ACCEL_SCALE_X;
    float corrected_ay = (raw_ay - ACCEL_BIAS_Y_MPS2) * ACCEL_SCALE_Y;
    float corrected_az = (raw_az - ACCEL_BIAS_Z_MPS2) * ACCEL_SCALE_Z;
    // Stationary/level gates below use corrected acceleration in g.
    float ax = corrected_ax / GRAVITY_MPS2;
    float ay = corrected_ay / GRAVITY_MPS2;
    float az = corrected_az / GRAVITY_MPS2;
    float gx = (float)Signed16(&data[9]) / 131.0f;
    float gy = (float)Signed16(&data[11]) / 131.0f;
    float gz = (float)Signed16(&data[13]) / 131.0f;
    accel_x_mps2 = corrected_ax;
    accel_y_mps2 = corrected_ay;
    accel_z_mps2 = corrected_az; // Still includes gravity; not world-frame acceleration.
    if (imu_state == 1) {
        robot_enable = 0;
        // Reject wheel motion, large angular rates, tilt or shaking at startup.
        float norm2 = ax * ax + ay * ay + az * az;
        if (encoder_moving || fabsf(gx) > 5.0f || fabsf(gy) > 5.0f ||
            fabsf(gz) > 5.0f || fabsf(ax) > 0.15f || fabsf(ay) > 0.15f ||
            az < 0.85f || norm2 < 0.81f || norm2 > 1.21f) {
            ResetCalibration();
            return;
        }
        bias_sum += gz;
        bias_square_sum += gz * gz;
        if (++imu_calibration_samples >= CALIBRATION_SAMPLES) {
            float mean = bias_sum / CALIBRATION_SAMPLES;
            float variance = bias_square_sum / CALIBRATION_SAMPLES - mean * mean;
            if (variance > 0.25f) { ResetCalibration(); return; }
            gyro_bias_z_dps = mean;
            imu_state = 2;
            gyro_filter_initialized = 0;
        }
        return;
    }
    float rate = (gz - gyro_bias_z_dps) * DEG_TO_RAD;
    if (!gyro_filter_initialized) {
        gyro_w_radps = rate;
        gyro_filter_initialized = 1;
    } else {
        float alpha = sample_dt / (0.03f + sample_dt);
        gyro_w_radps += alpha * (rate - gyro_w_radps);
    }
    yaw_relative_rad += gyro_w_radps * sample_dt;
    if (yaw_relative_rad > ROBOT_PI) yaw_relative_rad -= 2.0f * ROBOT_PI;
    if (yaw_relative_rad < -ROBOT_PI) yaw_relative_rad += 2.0f * ROBOT_PI;
}

void Robot_Update(uint32_t elapsed_ms, float left_rpm, float right_rpm,
                  int encoder_moving, float *left_target, float *right_target)
{
    float v, w;
    RobotForward(left_rpm, right_rpm, &v, &w);
    measured_v_mps = v;
    encoder_w_radps = w;
    ReadImu(encoder_moving);
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
