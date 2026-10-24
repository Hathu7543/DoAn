#ifndef TELEMETRY_UART_H
#define TELEMETRY_UART_H
#include "main.h"
#include <string.h>
#include "robot_control.h"

// USART1 PA9 TX / PA10 RX, 115200 8N1. Do not also initialize HAL USART1.
// RX interrupt only buffers bytes; main owns parsing and motor commands.
static uint8_t telemetry_frame[50];
static uint32_t telemetry_pos = 50, telemetry_tick, telemetry_sequence;
static volatile uint32_t telemetry_dropped;
static volatile uint8_t command_ring[128];
static volatile uint32_t command_head, command_tail, command_rx_error;
static uint8_t command_frame[12];
static unsigned command_used;
static uint32_t command_last_tick;
static uint8_t command_armed;
static volatile uint32_t command_valid, command_bad, command_timeout;

void USART1_IRQHandler(void)
{
    uint32_t status = USART1->SR;
    if (status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
        uint8_t byte = (uint8_t)USART1->DR; // SR then DR clears receive errors.
        if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
            command_rx_error = 1;
        } else {
            uint32_t next = (command_head + 1U) & 127U;
            if (next == command_tail) command_rx_error = 1;
            else {
                command_ring[command_head] = byte;
                command_head = next;
            }
        }
    }
}

static uint16_t Telemetry_Crc(const uint8_t *p, unsigned n)
{
    uint16_t crc = 0xffff;
    while (n--) {
        crc ^= (uint16_t)*p++ << 8;
        for (unsigned b = 0; b < 8; ++b)
            crc = (uint16_t)((crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1);
    }
    return crc;
}
static void Telemetry_Put(unsigned offset, uint64_t value, unsigned size)
{
    for (unsigned i = 0; i < size; ++i) {
        telemetry_frame[offset + i] = (uint8_t)value;
        value >>= 8;
    }
}
static void Command_Stop(void)
{
    command_v_mps = 0.0f;
    command_w_radps = 0.0f;
    robot_enable = 0;
}
static void Command_Byte(uint8_t byte)
{
    command_frame[command_used++] = byte;
    while (command_used) {
        int invalid = command_frame[0] != 0xaa ||
            (command_used >= 2 && command_frame[1] != 0xc3) ||
            (command_used >= 3 && command_frame[2] != 1) ||
            (command_used >= 4 && command_frame[3] != 12);
        if (!invalid && command_used < 12) return;
        if (!invalid) {
            uint16_t crc = (uint16_t)command_frame[10] | ((uint16_t)command_frame[11] << 8);
            if (crc == Telemetry_Crc(command_frame, 10)) {
                int16_t v, w;
                memcpy(&v, command_frame + 4, 2);
                memcpy(&w, command_frame + 6, 2);
                uint8_t enable = command_frame[8];
                command_used = 0;
                if (v < -100 || v > 100 || w < -500 || w > 500 ||
                    enable > 1 || command_frame[9] != 0) {
                    ++command_bad;
                    command_armed = 0;
                    Command_Stop();
                    return;
                }
                command_last_tick = HAL_GetTick();
                ++command_valid;
                if (!enable) {
                    Command_Stop();
                    command_armed = (imu_state == 2 && robot_fault == 0);
                } else if (command_armed && imu_state == 2 && robot_fault == 0) {
                    command_v_mps = (float)v * 0.001f;
                    command_w_radps = (float)w * 0.001f;
                    robot_enable = 1;
                }
                return;
            }
            ++command_bad;
        }
        --command_used;
        memmove(command_frame, command_frame + 1, command_used);
    }
}
static void Command_Service(void)
{
    // Check freshness BEFORE consuming queued commands; timeout requires STOP
    // to re-arm, so resumed motion heartbeats alone cannot restart the vehicle.
    if ((uint32_t)(HAL_GetTick() - command_last_tick) > 500U ||
        imu_state != 2 || robot_fault) {
        if (command_armed) ++command_timeout;
        command_armed = 0;
        Command_Stop();
    }
    if (command_rx_error) {
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        command_tail = command_head;
        command_rx_error = 0;
        __set_PRIMASK(mask);
        command_used = 0;
        command_armed = 0;
        Command_Stop();
    }
    for (unsigned i = 0; i < 128 && command_tail != command_head; ++i) {
        uint8_t byte = command_ring[command_tail];
        command_tail = (command_tail + 1U) & 127U;
        Command_Byte(byte);
    }
}
static void Telemetry_Float(unsigned offset, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    Telemetry_Put(offset, bits, 4);
}
static void Telemetry_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio);
    USART1->CR1 = 0;
    USART1->CR2 = 0;
    USART1->CR3 = 0;
    USART1->BRR = (HAL_RCC_GetPCLK2Freq() + 57600U) / 115200U;
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;
    HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    telemetry_tick = HAL_GetTick();
}
static void Telemetry_Service(void)
{
    Command_Service();
    if (telemetry_pos < sizeof(telemetry_frame) && (USART1->SR & USART_SR_TXE))
        USART1->DR = telemetry_frame[telemetry_pos++];
}
static void Telemetry_Send(uint32_t sample_ms, int64_t left, int64_t right,
                           float gyro, float ax, float ay, float az,
                           uint32_t imu, uint32_t fault, uint32_t enabled)
{
    if ((uint32_t)(sample_ms - telemetry_tick) < 20U) return;
    telemetry_tick = sample_ms;
    uint32_t seq = telemetry_sequence++;
    if (telemetry_pos < sizeof(telemetry_frame)) {
        ++telemetry_dropped;
        return;
    }
    telemetry_frame[0] = 0xaa;
    telemetry_frame[1] = 0x55;
    telemetry_frame[2] = 1;
    telemetry_frame[3] = sizeof(telemetry_frame);
    Telemetry_Put(4, seq, 4);
    Telemetry_Put(8, sample_ms, 4);
    Telemetry_Put(12, (uint64_t)left, 8);
    Telemetry_Put(20, (uint64_t)right, 8);
    Telemetry_Float(28, gyro);
    Telemetry_Float(32, ax);
    Telemetry_Float(36, ay);
    Telemetry_Float(40, az);
    telemetry_frame[44] = (uint8_t)imu;
    telemetry_frame[45] = (uint8_t)fault;
    telemetry_frame[46] = (uint8_t)enabled;
    telemetry_frame[47] = 0;
    Telemetry_Put(48, Telemetry_Crc(telemetry_frame, 48), 2);
    telemetry_pos = 0;
}
#endif
