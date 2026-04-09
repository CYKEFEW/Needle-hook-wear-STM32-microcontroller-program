#include "main.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart2;

void setTorque(float torque) {
    // 向 MotorUsart 发送力矩命令值
    char buf[32];
    snprintf(buf, sizeof(buf), "MZT%.3f\r\n", torque);
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen(buf), HAL_MAX_DELAY);
}

// 向 MotorUsart 发送字符串命令
void focCommand(const char* command) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%s\r\n", command);
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen(buf), HAL_MAX_DELAY);
}
