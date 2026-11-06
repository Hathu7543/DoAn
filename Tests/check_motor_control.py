"""Execute production motor/filter logic at compile time with mocked timers/GPIO."""
from pathlib import Path
import re
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
def read(path):
    return (root / path).read_text()
def strip_includes(code):
    return '\n'.join(line for line in code.splitlines() if not line.startswith('#include'))
def structure(code):
    return code[code.index('typedef struct'):code.index('} ', code.index('typedef struct'))] + code[code.index('} ', code.index('typedef struct')):].split(';', 1)[0] + ';\n'

config = strip_includes(read('Core/Inc/motor_config.h'))
types = structure(read('Core/Inc/speed_filter.h')) + structure(read('Core/Inc/motor_control.h'))
filter_code = strip_includes(read('Core/Src/speed_filter.c'))
filter_code = re.sub(r'^(void|float) (SpeedFilter_\w+)', r'constexpr \1 \2', filter_code, flags=re.M)
motor_code = strip_includes(read('Core/Src/motor_control.c')).replace('volatile ', '')
motor_code = re.sub(r'^(?:static )?(void|float) (\w+)\(', r'constexpr \1 \2(', motor_code, flags=re.M)
stub = r'''
#include <stdint.h>
constexpr float fabsf(float x) { return x < 0 ? -x : x; }
constexpr float fminf(float x, float y) { return x < y ? x : y; }
constexpr float fmaxf(float x, float y) { return x > y ? x : y; }
constexpr bool isfinite(float x) { return x == x && fabsf(x) <= 3.402823466e38f; }
constexpr bool near(float x, float y) { return fabsf(x-y) < 0.001f; }
struct TIM_HandleTypeDef { uint32_t count=0, arr=7199, compare[2]={}; };
struct GPIO_TypeDef { uint16_t pins=0; };
constexpr int GPIO_PIN_RESET=0, GPIO_PIN_SET=1;
constexpr void __HAL_TIM_SET_COUNTER(TIM_HandleTypeDef* t, uint32_t x) { t->count=x; }
constexpr uint32_t __HAL_TIM_GET_AUTORELOAD(TIM_HandleTypeDef* t) { return t->arr; }
constexpr void __HAL_TIM_SET_COMPARE(TIM_HandleTypeDef* t, uint32_t c, uint32_t x) { t->compare[c]=x; }
constexpr void HAL_GPIO_WritePin(GPIO_TypeDef* p, uint16_t mask, int value) {
    p->pins = value ? p->pins | mask : p->pins & ~mask;
}
'''
tests = r'''
}; // Simulation
constexpr bool filter_test() {
    SpeedFilter f={}; SpeedFilter_Reset(&f);
    if(!near(SpeedFilter_Update(&f,10),10))return false;
    if(!near(SpeedFilter_Update(&f,20),15))return false;
    SpeedFilter_Reset(&f);
    for(int i=0;i<10;i++)SpeedFilter_Update(&f,10);
    if(!near(SpeedFilter_Update(&f,20),11))return false;
    for(int i=0;i<9;i++)SpeedFilter_Update(&f,20);
    return near(f.sum,200) && f.sample_count==10;
}
constexpr bool drive_test() {
    Simulation s={}; TIM_HandleTypeDef enc={},pwm={}; GPIO_TypeDef gpio={};
    MotorController m={}; m.encoder_timer=&enc;m.pwm_timer=&pwm;
    m.direction_port=&gpio;m.in1_pin=1;m.in2_pin=2;m.drive_direction=1;m.encoder_direction=1;
    float duty=0; s.MotorController_Reset(&m);
    s.Motor_SetDuty(&m,50,&duty);
    if(pwm.compare[0]!=3600 || gpio.pins!=1 || !near(duty,50))return false;
    s.Motor_SetDuty(&m,-50,&duty);
    if(pwm.compare[0] || gpio.pins || duty)return false;
    s.Motor_SetDuty(&m,-50,&duty);
    if(pwm.compare[0]!=3600 || gpio.pins!=2 || !near(duty,-50))return false;
    s.Motor_SetDuty(&m,0,&duty);
    m.drive_direction=-1;m.pwm_channel=1;
    s.Motor_SetDuty(&m,150,&duty);
    return pwm.compare[1]==7200 && gpio.pins==2 && near(duty,100);
}
constexpr bool control_test() {
    Simulation s={}; TIM_HandleTypeDef enc={},pwm={}; GPIO_TypeDef gpio={};
    MotorController m={}; m.encoder_timer=&enc;m.pwm_timer=&pwm;
    m.direction_port=&gpio;m.in1_pin=1;m.in2_pin=2;m.drive_direction=1;m.encoder_direction=1;
    float measured=0,filtered=0,error=0,integral=0,duty=0,avg=0;
    s.MotorController_Reset(&m);m.previous_count=65530;
    s.Motor_Update(&m,4,10,30,&measured,&filtered,&error,&integral,&duty,&avg);
    if(m.total_counts!=10 || !near(measured,45.454545f))return false;
    s.Motor_Update(&m,65530,10,0,&measured,&filtered,&error,&integral,&duty,&avg);
    if(m.total_counts || duty || integral || error || pwm.compare[0])return false;
    s.MotorController_Reset(&m);m.encoder_direction=-1;
    s.Motor_Update(&m,65526,10,30,&measured,&filtered,&error,&integral,&duty,&avg);
    if(m.total_counts!=10 || !near(measured,45.454545f))return false;
    s.MotorController_Reset(&m);m.encoder_direction=1;
    m.integral_output=99;
    s.Motor_Update(&m,0,10,200,&measured,&filtered,&error,&integral,&duty,&avg);
    if(!near(integral,99) || !near(duty,100))return false; // anti-windup
    s.MotorController_Reset(&m);
    s.Motor_Update(&m,1320,1000,0,&measured,&filtered,&error,&integral,&duty,&avg);
    if(!near(avg,60) || !near(measured,60))return false;
    return near(s.Proportional_Update(1,.6f,.15f),.15f) &&
           near(s.Proportional_Update(3,.6f,.15f),.9f);
}
static_assert(filter_test(), "filter startup/window wrap/reset");
static_assert(drive_test(), "PWM scaling, clamp, channel and reversal blanking");
static_assert(control_test(), "encoder rollover/sign, stop, PI and report");
'''
out=Path(__file__).with_name('motor_control_compile_test.cpp')
out.write_text('\n'.join([stub, config, types, filter_code, 'struct Simulation {', motor_code, tests]))
subprocess.run([sys.argv[1], '-std=c++17', '-fsyntax-only', '-Wall', '-Wextra', '-Werror', str(out)], check=True)
print('PASS: moving average, PWM channels/polarity/reversal, encoder wrap/sign, stop, anti-windup, 1s RPM.')
