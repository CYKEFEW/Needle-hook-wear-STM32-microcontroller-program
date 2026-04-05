#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* =========================
 * USART1: 终端命令接收
 * ========================= */
static uint8_t usart1_cmd_buf[512];
static uint8_t usart1_cmd_idx = 0;
volatile char usart1_rx_byte = 0;
volatile char usart2_rx_byte = 0;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

volatile uint8_t ConMode = 0; // 0:张力控制模式 1:速度控制模式
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* USART1：命令行 */
    if (huart == &huart1)
    {
        if (usart1_rx_byte == '\n')
        {
            usart1_cmd_buf[usart1_cmd_idx] = '\0';

            if (strncmp((char *)usart1_cmd_buf, "Con ", 4) == 0)
            {
                if (ConMode == 1){
                    float value = atof((char *)usart1_cmd_buf + 4);
                    control(value);
                } else {
                    printf("Please set ConMode to 1 (Speed Control Mode) before using Con command.\n");
                }
            }
            else if (strncmp((char *)usart1_cmd_buf, "Forward", 7) == 0)
            {
                forward();
            }
            else if (strncmp((char *)usart1_cmd_buf, "Backward", 8) == 0)
            {
                backward();
            }
            else if (strncmp((char *)usart1_cmd_buf, "Enable", 6) == 0)
            {
                enable();
            }
            else if (strncmp((char *)usart1_cmd_buf, "Disable", 7) == 0)
            {
                disable();
            }
            else if (strncmp((char *)usart1_cmd_buf, "Data", 4) == 0){
                if (ConMode == 0){
                    float CH1,CH2,target_rpm;
                    sscanf((char *)usart1_cmd_buf, "Data %f %f %f",&CH1,&CH2,&target_rpm);
                    pid(CH1,CH2,target_rpm);
                }
            }
            else if (strncmp((char *)usart1_cmd_buf, "F", 1) == 0){
                if (ConMode == 0){
                    extern PID Force_Pid;
                    float target = 0.0f;
                    sscanf((char *)usart1_cmd_buf, "F %f",&target);
                    Force_Pid.target = target;
                    printf("Set Force target:%.2f\n",target);
                } else {
                    printf("Please set ConMode to 0 (Force Control Mode) before using F command.\n");
                }
            }
            else if (strncmp((char *)usart1_cmd_buf, "pid", 3) == 0){
                extern PID Force_Pid;
                float kp,ki,kd;
                sscanf((char *)usart1_cmd_buf, "pid %f %f %f",&kp,&ki,&kd);
                Force_Pid.Kp = kp;
                Force_Pid.Ki = ki;
                Force_Pid.Kd = kd;
                printf("Set PID: Kp=%.4f Ki=%.4f Kd=%.4f\n",kp,ki,kd);
            }
            else if (strncmp((char *)usart1_cmd_buf, "ConMode", 7) == 0){
                sscanf((char *)usart1_cmd_buf, "ConMode %hhu",&ConMode);
                printf("Set Control Mode:%s\n",ConMode==0?"Force":"Speed");
            }
            else
            {
                printf("Unknown command: %s\n", usart1_cmd_buf);
            }

            usart1_cmd_idx = 0;
            memset(usart1_cmd_buf, 0, sizeof(usart1_cmd_buf));
        }
        else
        {
            if (usart1_rx_byte != '\r')
            {
                if (usart1_cmd_idx < (sizeof(usart1_cmd_buf) - 1u))
                {
                    usart1_cmd_buf[usart1_cmd_idx++] = (uint8_t)usart1_rx_byte;
                }
                else
                {
                    usart1_cmd_idx = 0;
                    memset(usart1_cmd_buf, 0, sizeof(usart1_cmd_buf));
                    printf("Buffer overflow!\n");
                }
            }
        }

        HAL_UART_Receive_IT(&huart1, (uint8_t *)&usart1_rx_byte, 1);
        return;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* 清除错误标志 */
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);

    if (huart == &huart1)
    {
        usart1_cmd_idx = 0;
        memset(usart1_cmd_buf, 0, sizeof(usart1_cmd_buf));
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&usart1_rx_byte, 1);
        printf("TensionUsart error!\n");
    }
    else if (huart == &huart2)
    {
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&usart2_rx_byte, 1);
        printf("MotorUsart error!\n");
    }

}
