/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define MOTOR_TX_Pin GPIO_PIN_2
#define MOTOR_TX_GPIO_Port GPIOA
#define MOTOR_RX_Pin GPIO_PIN_3
#define MOTOR_RX_GPIO_Port GPIOA
#define PID_LED_Pin GPIO_PIN_6
#define PID_LED_GPIO_Port GPIOC
#define PUL_Pin GPIO_PIN_8
#define PUL_GPIO_Port GPIOA
#define TENSION_TX_Pin GPIO_PIN_9
#define TENSION_TX_GPIO_Port GPIOA
#define TENSION_RX_Pin GPIO_PIN_10
#define TENSION_RX_GPIO_Port GPIOA
#define ENA_Pin GPIO_PIN_11
#define ENA_GPIO_Port GPIOA
#define DIR_Pin GPIO_PIN_12
#define DIR_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
// 电机控制相关定义
void control(float rpm);
void silentcontrol(float rpm);
void setTorque(float torque);
void forward(void);
void backward(void);
void enable(void);
void disable(void);

// pid相关定义
void pid(float CH1, float CH2, float target_rpm);
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

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
