#include "main.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart2;

void setTension(float tension) {
    // 向MotorUsart发送张力值
    char buf[32];
    snprintf(buf, sizeof(buf), "Tension: %.2f\r", tension);
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen(buf), HAL_MAX_DELAY);
}
