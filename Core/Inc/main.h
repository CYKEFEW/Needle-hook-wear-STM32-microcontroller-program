/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PID_LED_Pin GPIO_PIN_6
#define PID_LED_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_7
#define LED_GPIO_Port GPIOC
#define sampleKey_Pin GPIO_PIN_8
#define sampleKey_GPIO_Port GPIOC
#define sampleKey_EXTI_IRQn EXTI9_5_IRQn
#define ZeroKey_Pin GPIO_PIN_9
#define ZeroKey_GPIO_Port GPIOC
#define ZeroKey_EXTI_IRQn EXTI9_5_IRQn
#define PUL_Pin GPIO_PIN_8
#define PUL_GPIO_Port GPIOA
#define ENA_Pin GPIO_PIN_11
#define ENA_GPIO_Port GPIOA
#define DIR_Pin GPIO_PIN_12
#define DIR_GPIO_Port GPIOA
#define DIR2_Pin GPIO_PIN_4
#define DIR2_GPIO_Port GPIOB
#define ENA2_Pin GPIO_PIN_5
#define ENA2_GPIO_Port GPIOB
#define PUL2_Pin GPIO_PIN_6
#define PUL2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define DATA_NUM 2 // 数据个数

void Senddata(void);
void control(float rpm);
void control2(float rpm);
void silentcontrol(float rpm);

void erase_flash(void);
void write_flash(float data[DATA_NUM]);
float read_flash(uint8_t index);

void zero(void);

void delay_ms(uint32_t ms);

void forward(void);
void backward(void);
void enable(void);
void disable(void);
void forward2(void);
void backward2(void);
void enable2(void);
void disable2(void);

void USART2_ModbusInit(void);
void USART2_ModbusPoll(void);

void pid(float CH1, float CH2);
typedef struct {
    // 增量式pid
    float Kp;
    float Ki;
    float Kd;
    float error;
    float prev_error;
    float prev_prev_error;
    float output;
    float target;
} PID;

typedef struct
{
    uint8_t  ok;          /* 数据是否有效 */
    uint8_t  addr;
    int16_t  py1_raw;
    int16_t  py2_raw;
    uint8_t  dp1;
    uint8_t  dp2;
    float    py1;
    float    py2;
} Modbus2Data;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
