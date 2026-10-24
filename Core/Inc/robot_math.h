#ifndef ROBOT_MATH_H
#define ROBOT_MATH_H

#define ROBOT_RADIUS_M 0.0325f
#define ROBOT_TRACK_M 0.215f
#define ROBOT_MASS_KG 2.0f
#define ROBOT_PI 3.14159265358979323846f
#define ROBOT_MAX_V 0.1f
#define ROBOT_MAX_W 0.5f
#define ROBOT_ACCEL_MPS2 0.15f
#define ROBOT_ACCEL_RADPS2 0.8f
#define ROBOT_MAX_RPM 60.0f

static inline float RobotClamp(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

static inline float RobotSlew(float previous, float target, float step)
{
    return previous + RobotClamp(target - previous, -step, step);
}

static inline void RobotInverse(float v, float w, float *left, float *right)
{
    const float factor = 60.0f / (2.0f * ROBOT_PI * ROBOT_RADIUS_M);
    *left = (v - 0.5f * ROBOT_TRACK_M * w) * factor;
    *right = (v + 0.5f * ROBOT_TRACK_M * w) * factor;
    float l = *left < 0.0f ? -*left : *left;
    float r = *right < 0.0f ? -*right : *right;
    float peak = l > r ? l : r;
    if (peak > ROBOT_MAX_RPM) {
        *left *= ROBOT_MAX_RPM / peak;
        *right *= ROBOT_MAX_RPM / peak;
    }
}

static inline void RobotForward(float left, float right, float *v, float *w)
{
    const float factor = 2.0f * ROBOT_PI * ROBOT_RADIUS_M / 60.0f;
    *v = (left + right) * 0.5f * factor;
    *w = (right - left) * factor / ROBOT_TRACK_M;
}
#endif
