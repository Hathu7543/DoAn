"""Compile-time tests of production robot C logic with mocked I2C/time.
Only qualifiers/local initialization are adapted for C++17 constexpr execution.
No motor or physical I2C access. Run with ARM g++ path as argument.
"""
from pathlib import Path
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
math_code = (root / 'Core/Inc/robot_math.h').read_text().replace('static inline', 'static constexpr')
app = (root / 'Core/Src/robot_control.c').read_text() + chr(10) + (root / 'Core/Src/imu_mpu6050.c').read_text()
app = '\n'.join(line for line in app.splitlines() if not line.startswith('#include'))
app = app.replace('volatile ', '').replace('static ', '')
for signature in ['int WriteRegister', 'int16_t Signed16', 'void ImuFault',
                  'void Robot_Init', 'void Imu_Init', 'void ResetCalibration', 'void Imu_Update', 'void Robot_Update']:
    app = app.replace(signature, 'constexpr ' + signature)
app = app.replace('uint8_t data[15];', 'uint8_t data[15] = {};')
app = app.replace('float v, w;', 'float v = 0, w = 0;')

stub = r'''
#include <stdint.h>
constexpr int HAL_OK=0, I2C_MEMADD_SIZE_8BIT=1;
struct I2C_HandleTypeDef {};
constexpr float fabsf(float x) { return x < 0 ? -x : x; }
constexpr bool isfinite(float x) { return x == x && x <= 3.402823466e38f && x >= -3.402823466e38f; }
struct Simulation {
    uint32_t tick=0;
    bool io_failure=false, data_ready=true;
    int16_t raw_z=0;
    float accel_input[3]={0.15f,0.1f,11.6f}; // Measured module bias, upright.
    I2C_HandleTypeDef bus={};
    constexpr uint32_t HAL_GetTick() { return tick; }
    constexpr void HAL_Delay(uint32_t ms) { tick += ms; }
    constexpr int HAL_I2C_Mem_Write(I2C_HandleTypeDef*, int, int, int, uint8_t*, int, int) {
        return io_failure ? 1 : HAL_OK;
    }
    constexpr int HAL_I2C_Mem_Read(I2C_HandleTypeDef*, int, int reg, int, uint8_t* data, int len, int) {
        if (io_failure) return 1;
        for (int i=0;i<len;i++) data[i]=0;
        if (reg==0x75) { data[0]=0x68; return HAL_OK; }
        data[0]=data_ready ? 1 : 0;
        for(int axis=0;axis<3;axis++) {
            int16_t raw=(int16_t)(accel_input[axis]*16384.0f/9.80665f);
            data[1+2*axis]=(uint16_t)raw >> 8;
            data[2+2*axis]=(uint16_t)raw & 255;
        }
        data[13]=(uint16_t)raw_z >> 8;
        data[14]=(uint16_t)raw_z & 255;
        return HAL_OK;
    }
'''
checks = r'''
};
constexpr bool near(float a, float b, float tol=0.001f) { return fabsf(a-b)<tol; }
constexpr bool kinematics() {
    float l=0,r=0,v=0,w=0;
    RobotInverse(0.1f,0,&l,&r);
    if (!near(l,29.38245f) || !near(r,l)) return false;
    RobotInverse(0,0.5f,&l,&r);
    if (!near(l,-15.79307f) || !near(r,15.79307f)) return false;
    RobotForward(l,r,&v,&w);
    if (!near(v,0) || !near(w,0.5f)) return false;
    RobotInverse(-0.1f,-0.5f,&l,&r);
    RobotForward(l,r,&v,&w);
    if (!near(v,-0.1f) || !near(w,-0.5f)) return false;
    RobotInverse(1,2,&l,&r);
    if (fabsf(l)>60.001f || fabsf(r)>60.001f) return false;
    return near(RobotSlew(0,0.1f,0.0015f),0.0015f) &&
           near(RobotSlew(0.05f,-0.1f,0.0015f),0.0485f);
}
constexpr bool lifecycle() {
    Simulation s={};
    s.Robot_Init(&s.bus);
    if(s.imu_state!=1 || s.robot_enable) return false;
    float l=0,r=0;
    s.raw_z=131; // Known stationary bias = 1 degree/s.
    for(int i=0;i<150;i++) { s.tick+=10; s.Robot_Update(10,0,0,0,&l,&r); }
    s.tick+=10; s.Robot_Update(10,0,0,1,&l,&r); // Encoder movement resets calibration.
    if(s.imu_calibration_samples!=0) return false;
    for(int i=0;i<300;i++) { s.tick+=10; s.Robot_Update(10,0,0,0,&l,&r); }
    if(s.imu_state!=2 || s.robot_enable || !near(s.gyro_bias_z_dps,1)) return false;
    s.robot_enable=1;
    for(int i=0;i<100;i++) { s.tick+=10; s.Robot_Update(10,0,0,0,&l,&r); }
    if(!near(l,29.38245f) || !near(r,l) || s.dynamics_valid) return false;
    s.command_v_mps=0; s.command_w_radps=0;
    s.raw_z=655; s.tick+=10; s.Robot_Update(10,0,0,0,&l,&r);
    if(l!=0 || r!=0) return false; // Zero command must override yaw correction.
    s.command_w_radps=0.5f; s.raw_z=131; s.body_inertia_kgm2=0.02f;
    s.tick+=10; s.Robot_Update(10,0,0,0,&l,&r);
    if(!s.dynamics_valid || s.wheel_torque_left_nm>=0 || s.wheel_torque_right_nm<=0) return false;
    s.io_failure=true; s.tick+=10; s.Robot_Update(10,0,0,0,&l,&r);
    if(s.imu_state!=3 || !s.robot_fault || s.robot_enable || l || r) return false;
    s.io_failure=false; s.robot_enable=1; s.tick+=10; s.Robot_Update(10,0,0,0,&l,&r);
    return !s.robot_enable && l==0 && r==0; // Fault remains latched.
}
constexpr bool faults() {
    Simulation s={}; float l=0,r=0;
    s.io_failure=true; s.Robot_Init(&s.bus);
    if(s.imu_state!=3) return false;
    Simulation t={}; t.Robot_Init(&t.bus);
    t.data_ready=false; t.tick+=100; t.Robot_Update(10,0,0,0,&l,&r);
    if(t.imu_state!=3) return false;
    Simulation u={}; u.Robot_Init(&u.bus); u.tick+=60; u.Robot_Update(60,0,0,0,&l,&r);
    return u.robot_fault==2 && !u.robot_enable && l==0 && r==0;
}
static_assert(kinematics(), "kinematics / slew / saturation failed");
constexpr bool accelerationCalibration() {
    const float readings[6]={9.8f,-9.5f,9.9f,-9.7f,11.6f,-8.0f};
    for(int pose=0;pose<6;pose++) {
        Simulation s={};
        s.Robot_Init(&s.bus);
        s.accel_input[2]=1.8f;
        s.accel_input[pose/2]=readings[pose];
        s.tick+=10;
        float l=0,r=0;
        s.Robot_Update(10,0,0,0,&l,&r);
        float corrected=pose/2==0 ? s.accel_x_mps2 :
                        pose/2==1 ? s.accel_y_mps2 : s.accel_z_mps2;
        if(!near(corrected,pose%2==0 ? 9.80665f : -9.80665f)) return false;
        if(pose!=4 && s.imu_calibration_samples!=0) return false;
        if(l || r || s.robot_enable) return false;
    }
    return true;
}
static_assert(accelerationCalibration(), "six-pose calibration / orientation rejection failed");
static_assert(lifecycle(), "calibration / enable / stop / I2C fault failed");
static_assert(faults(), "startup / stale sample / overrun fault failed");
'''
out = Path(__file__).with_name('robot_control_compile_test.cpp')
out.write_text(math_code + stub + app + checks)
subprocess.run([sys.argv[1], '-std=c++17', '-fsyntax-only', '-Wall', '-Wextra', '-Werror', str(out)], check=True)
print('PASS: kinematics, calibration with movement rejection, ramp, stop, dynamics, fault latching.')
