#include "main.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart2;

void setTorque(float torque) {
    // 向MotorUsart发送转矩值
    char buf[32];
    snprintf(buf, sizeof(buf), "Torque: %.2f\r", torque);
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen(buf), HAL_MAX_DELAY);
}
