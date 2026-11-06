#ifndef TELEMETRY_UART_H
#define TELEMETRY_UART_H
#include <stdint.h>

// USART1 PA9 TX / PA10 RX, 115200 8N1. This module owns USART1.
void Telemetry_Init(void);
void Telemetry_Service(void);
void Telemetry_RxIRQHandler(void);
void Telemetry_Send(uint32_t sample_ms, int64_t left, int64_t right,
                    float gyro, float ax, float ay, float az,
                    uint32_t imu, uint32_t fault, uint32_t enabled);
#endif
