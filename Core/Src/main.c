/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#define SPEED_AVERAGE_LENGTH 10U
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
    TIM_HandleTypeDef *encoder_timer;
    uint32_t pwm_channel;
    GPIO_TypeDef *direction_port;
    uint16_t in1_pin;
    uint16_t in2_pin;
    int8_t encoder_direction;
    int8_t drive_direction;
    uint16_t previous_count;
    float speed_buffer[SPEED_AVERAGE_LENGTH];
    float speed_sum;
    uint16_t speed_index;
    uint16_t speed_sample_count;
    float previous_speed;
    float integral_output;
    int32_t report_counts;
    uint32_t report_elapsed_ms;
    int8_t applied_direction;
    int8_t target_direction;
} MotorController;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CONTROL_PERIOD_MS 10U
#define P_FINE_BAND_RPM 2.0f
#define SPEED_REPORT_PERIOD_MS 1000U
// Counts per output-shaft revolution in TI12 mode: verify on your motor.
#define ENCODER_COUNTS_PER_REV 1320.0f
// Moving average over 10 speed samples: 100 ms window at a 10 ms period.
// Larger windows smooth more but delay the speed feedback.
// At 1320 counts/rev, one count over this window represents about 0.455 RPM.
#if SPEED_AVERAGE_LENGTH < 1U || SPEED_AVERAGE_LENGTH > 200U
#error "SPEED_AVERAGE_LENGTH must be between 1 and 200"
#endif
// Positive RPM means vehicle-forward for both mirrored wheels.
#define LEFT_ENCODER_DIRECTION   1
#define RIGHT_ENCODER_DIRECTION -1
#define LEFT_DRIVE_DIRECTION     1
#define RIGHT_DRIVE_DIRECTION   -1
#define TARGET_STOP_EPSILON_RPM 0.01f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
volatile float target_left_rpm = 30.0f;
volatile float target_right_rpm = 30.0f;
volatile float measured_left_rpm = 0.0f;
volatile float measured_right_rpm = 0.0f;
volatile float filtered_left_rpm = 0.0f;
volatile float filtered_right_rpm = 0.0f;
volatile float pwm_left = 0.0f;   // Signed duty: positive means vehicle-forward.
volatile float pwm_right = 0.0f;
volatile float error_left_rpm = 0.0f;
volatile float error_right_rpm = 0.0f;
volatile float integral_left_duty = 0.0f;
volatile float integral_right_duty = 0.0f;
volatile float rpm_average_left_1s = 0.0f;
volatile float rpm_average_right_1s = 0.0f;
// Gains expressed in percent duty, rather than raw timer counts.
// Reduce proportional sensitivity to quantized speed feedback and MA delay.
// These are trial gains; validate settling and load response on the motor.
volatile float Kp = 0.6f;
volatile float KpFine = 0.15f; // P slope inside +/-2 RPM; Ki still sees full error.
volatile float Ki = 1.0f;
volatile float Kd = 0.0f;
static MotorController left_motor;
static MotorController right_motor;
static uint32_t last_control_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void MotorController_Reset(MotorController *motor)
{
    for (uint16_t i = 0; i < SPEED_AVERAGE_LENGTH; ++i) {
        motor->speed_buffer[i] = 0.0f;
    }
    motor->speed_sum = 0.0f;
    motor->speed_index = 0;
    motor->speed_sample_count = 0;
    motor->previous_speed = 0.0f;
    motor->integral_output = 0.0f;
    motor->report_counts = 0;
    motor->report_elapsed_ms = 0;
    motor->applied_direction = 0;
    motor->target_direction = 0;
    __HAL_TIM_SET_COUNTER(motor->encoder_timer, 0);
    motor->previous_count = 0;
}

static float SpeedAverage_Update(MotorController *motor, float input)
{
    motor->speed_sum -= motor->speed_buffer[motor->speed_index];
    motor->speed_buffer[motor->speed_index] = input;
    motor->speed_sum += input;
    if (++motor->speed_index >= SPEED_AVERAGE_LENGTH) {
        motor->speed_index = 0;
    }
    if (motor->speed_sample_count < SPEED_AVERAGE_LENGTH) {
        ++motor->speed_sample_count;
    }
    return motor->speed_sum / (float)motor->speed_sample_count;
}

static float Clamp(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static float Proportional_Update(float error, float kp, float kp_fine)
{
    // Continuous piecewise-linear P: no dead band, no switch in output.
    // Small encoder steps cause smaller duty changes near the setpoint.
    kp = fmaxf(kp, 0.0f);
    kp_fine = Clamp(kp_fine, 0.0f, kp);
    float magnitude = fabsf(error);
    float fine_error = fminf(magnitude, P_FINE_BAND_RPM);
    float output = kp_fine * fine_error + kp * (magnitude - fine_error);
    return (error < 0.0f) ? -output : output;
}

static void SpeedReport_Update(MotorController *motor,
                               int32_t counts,
                               uint32_t elapsed_ms,
                               volatile float *rpm_average)
{
    motor->report_counts += counts;
    motor->report_elapsed_ms += elapsed_ms;
    if (motor->report_elapsed_ms >= SPEED_REPORT_PERIOD_MS) {
        *rpm_average = (float)motor->report_counts * 60000.0f /
                       (ENCODER_COUNTS_PER_REV * (float)motor->report_elapsed_ms);
        motor->report_counts = 0;
        motor->report_elapsed_ms = 0;
    }
}

static void Motor_SetDuty(MotorController *motor,
                          float logical_duty,
                          volatile float *pwm_value)
{
    logical_duty = Clamp(logical_duty, -100.0f, 100.0f);
    float physical_duty = logical_duty * (float)motor->drive_direction;
    float magnitude = fabsf(physical_duty);
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim4) + 1U;
    uint32_t compare = (uint32_t)(magnitude * (float)period / 100.0f + 0.5f);
    int8_t desired_direction = (physical_duty > 0.0f) ? 1 :
                               ((physical_duty < 0.0f) ? -1 : 0);

    // Remove PWM before changing H-bridge direction.
    if (desired_direction != 0 && motor->applied_direction != 0 &&
        desired_direction != motor->applied_direction) {
        __HAL_TIM_SET_COMPARE(&htim4, motor->pwm_channel, 0);
    }

    if (compare == 0U) {
        HAL_GPIO_WritePin(motor->direction_port,
                          motor->in1_pin | motor->in2_pin, GPIO_PIN_RESET);
    } else if (physical_duty > 0.0f) {
        HAL_GPIO_WritePin(motor->direction_port, motor->in2_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->direction_port, motor->in1_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(motor->direction_port, motor->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->direction_port, motor->in2_pin, GPIO_PIN_SET);
    }
    __HAL_TIM_SET_COMPARE(&htim4, motor->pwm_channel, compare);
    motor->applied_direction = desired_direction;
    *pwm_value = (logical_duty < 0.0f ? -1.0f : 1.0f) *
                 (100.0f * (float)compare / (float)period);
}

static void Motor_Update(MotorController *motor,
                         uint32_t elapsed_ms,
                         float setpoint,
                         volatile float *measured_rpm,
                         volatile float *filtered_rpm,
                         volatile float *error_rpm,
                         volatile float *integral_duty,
                         volatile float *pwm_value,
                         volatile float *rpm_average)
{
    float dt = (float)elapsed_ms * 0.001f;
    uint16_t count_now = (uint16_t)__HAL_TIM_GET_COUNTER(motor->encoder_timer);
    int32_t delta_count = (int32_t)count_now - (int32_t)motor->previous_count;
    if (delta_count > 32767) delta_count -= 65536;
    if (delta_count < -32768) delta_count += 65536;
    motor->previous_count = count_now;

    int32_t directed_counts = motor->encoder_direction * delta_count;
    SpeedReport_Update(motor, directed_counts, elapsed_ms, rpm_average);
    float speed = (float)directed_counts * 60.0f /
                  (ENCODER_COUNTS_PER_REV * dt);
    *measured_rpm = speed;
    float filtered = SpeedAverage_Update(motor, speed);
    float derivative = -(filtered - motor->previous_speed) / dt;
    motor->previous_speed = filtered;
    *filtered_rpm = filtered;

    if (!isfinite(setpoint) || fabsf(setpoint) <= TARGET_STOP_EPSILON_RPM) {
        motor->integral_output = 0.0f;
        *integral_duty = 0.0f;
        *error_rpm = 0.0f;
        motor->target_direction = 0;
        Motor_SetDuty(motor, 0.0f, pwm_value);
        return;
    }

    int8_t requested_direction = (setpoint > 0.0f) ? 1 : -1;
    if (motor->target_direction != 0 &&
        requested_direction != motor->target_direction) {
        motor->integral_output = 0.0f;
    }
    motor->target_direction = requested_direction;

    float error = setpoint - filtered;
    *error_rpm = error;
    float kp = Kp;
    float kp_fine = KpFine;
    float ki = Ki;
    float kd = Kd;
    float output_without_i = Proportional_Update(error, kp, kp_fine) +
                             kd * derivative;
    if (ki > 0.0f) {
        float change = ki * error * dt;
        float candidate = Clamp(motor->integral_output + change, -100.0f, 100.0f);
        float candidate_output = output_without_i + candidate;
        if ((candidate_output >= -100.0f && candidate_output <= 100.0f) ||
            (candidate_output > 100.0f && change < 0.0f) ||
            (candidate_output < -100.0f && change > 0.0f)) {
            motor->integral_output = candidate;
        }
    } else {
        motor->integral_output = 0.0f;
    }
    *integral_duty = motor->integral_output;
    Motor_SetDuty(motor, output_without_i + motor->integral_output, pwm_value);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  left_motor.encoder_timer = &htim3;
  left_motor.pwm_channel = TIM_CHANNEL_1;
  left_motor.direction_port = GPIOA;
  left_motor.in1_pin = GPIO_PIN_1;
  left_motor.in2_pin = GPIO_PIN_2;
  left_motor.encoder_direction = LEFT_ENCODER_DIRECTION;
  left_motor.drive_direction = LEFT_DRIVE_DIRECTION;

  right_motor.encoder_timer = &htim2;
  right_motor.pwm_channel = TIM_CHANNEL_2;
  right_motor.direction_port = GPIOB;
  right_motor.in1_pin = GPIO_PIN_0;
  right_motor.in2_pin = GPIO_PIN_1;
  right_motor.encoder_direction = RIGHT_ENCODER_DIRECTION;
  right_motor.drive_direction = RIGHT_DRIVE_DIRECTION;

  MotorController_Reset(&left_motor);
  MotorController_Reset(&right_motor);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  Motor_SetDuty(&left_motor, 0.0f, &pwm_left);
  Motor_SetDuty(&right_motor, 0.0f, &pwm_right);

  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK ||
      HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK) {
      Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2) != HAL_OK) {
      Error_Handler();
  }
  last_control_tick = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();
    uint32_t elapsed_ms = now - last_control_tick;
    if (elapsed_ms >= CONTROL_PERIOD_MS) {
        last_control_tick = now;
        Motor_Update(&left_motor, elapsed_ms, target_left_rpm,
                     &measured_left_rpm, &filtered_left_rpm,
                     &error_left_rpm, &integral_left_duty,
                     &pwm_left, &rpm_average_left_1s);
        Motor_Update(&right_motor, elapsed_ms, target_right_rpm,
                     &measured_right_rpm, &filtered_right_rpm,
                     &error_right_rpm, &integral_right_duty,
                     &pwm_right, &rpm_average_right_1s);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 7199;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  if (htim4.Instance == TIM4) {
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  }
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
