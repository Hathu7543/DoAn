"""Compile-time execution of production STM32 command parser with mocked IO/time."""
from pathlib import Path
import subprocess
import sys

source = (Path(__file__).resolve().parents[1] / 'Core/Src/telemetry_uart.c').read_text()
crc = source[source.index('static uint16_t Telemetry_Crc'):source.index('static void Telemetry_Put')]
parser = source[source.index('static void Command_Stop'):source.index('static void Telemetry_Float')]
code = (crc + parser).replace('static ', 'constexpr ')
code = code.replace('int16_t v, w;', 'int16_t v = 0, w = 0;')
stub = r'''
#include <stdint.h>
struct Simulation {
uint8_t command_frame[12]={}, command_ring[128]={};
unsigned command_used=0;
uint32_t command_head=0,command_tail=0,command_rx_error=0,command_last_tick=0;
uint32_t command_valid=0,command_bad=0,command_timeout=0, tick=100;
uint8_t command_armed=0;
uint32_t robot_enable=0,imu_state=2,robot_fault=0;
float command_v_mps=0,command_w_radps=0;
constexpr uint32_t HAL_GetTick(){return tick;}
constexpr uint32_t __get_PRIMASK(){return 0;}
constexpr void __disable_irq(){}
constexpr void __set_PRIMASK(uint32_t){}
constexpr void memcpy(int16_t* out,const uint8_t* p,unsigned){
    int value=p[0]+256*p[1]; *out=(int16_t)(value>=32768?value-65536:value);
}
constexpr void memmove(uint8_t* out,const uint8_t* p,unsigned n){
    for(unsigned i=0;i<n;++i)out[i]=p[i];
}
'''
tests = r'''
constexpr void packet(int v,int w,int en,bool corrupt=false){
    uint8_t p[12]={0xaa,0xc3,1,12,(uint8_t)v,(uint8_t)((uint16_t)v>>8),
       (uint8_t)w,(uint8_t)((uint16_t)w>>8),(uint8_t)en,0,0,0};
    uint16_t c=Telemetry_Crc(p,10); p[10]=(uint8_t)c; p[11]=(uint8_t)(c>>8);
    if(corrupt)p[4]^=1;
    for(auto b:p){command_ring[command_head]=b;command_head=(command_head+1)&127;}
    Command_Service();
}
};
constexpr bool checks(){
 Simulation s;
 const uint8_t vector[]={'1','2','3','4','5','6','7','8','9'};
 if(s.Telemetry_Crc(vector,9)!=0x29b1)return false;
 s.packet(50,0,1); if(s.robot_enable)return false; // no arm
 s.packet(0,0,0);s.packet(50,-300,1);
 if(!s.robot_enable || s.command_v_mps<0.049f || s.command_w_radps> -0.299f)return false;
 uint32_t valid=s.command_valid;
 s.packet(0,0,0,true);if(s.command_valid!=valid)return false;
 s.tick+=501;s.Command_Service();if(s.robot_enable||s.command_armed)return false;
 s.packet(50,0,1);if(s.robot_enable)return false; // heartbeat cannot rearm
 s.packet(0,0,0);s.packet(-50,300,1);if(!s.robot_enable)return false;
 s.packet(101,0,1);if(s.robot_enable||s.command_armed)return false;
 s.packet(0,0,0);s.packet(50,0,1);s.command_rx_error=1;s.Command_Service();
 if(s.robot_enable||s.command_armed)return false;
 s.imu_state=1;s.packet(0,0,0);s.packet(50,0,1);if(s.robot_enable)return false;
 s.imu_state=2;s.packet(0,0,0);s.packet(50,0,1);s.robot_fault=1;s.Command_Service();
 if(s.robot_enable)return false;
 s.robot_fault=0;s.packet(0,0,0);s.packet(0,-500,1);if(!s.robot_enable)return false;
 s.packet(0,0,0);return !s.robot_enable;
}
static_assert(checks(),"UART command checks failed");
'''
target = Path(__file__).with_name('velocity_uart_compile_test.cpp')
target.write_text(stub + code + tests)
subprocess.run([sys.argv[1], '-std=c++17', '-fsyntax-only', str(target)], check=True)
print('PASS: CRC vector, signed commands, corruption, limits, timeout, rearm, RX errors, IMU/fault interlock, stop')
