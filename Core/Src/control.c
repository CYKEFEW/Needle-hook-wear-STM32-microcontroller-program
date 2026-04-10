#include "main.h"
#include "stdio.h"

float stepAngle = 0.225f;
uint16_t rpmMAX = 1500;

extern TIM_HandleTypeDef htim1;
void control(float rpm)
{
    // rpm为0.6~600，对应定时器周期50000~100
    if (rpm >= 0)
    {
        if (rpm < 0.6)
        {
            // 停止TIM1
            HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
            // printf("Stop\n");
            return;
        }
        if (rpm > rpmMAX)
        {
            rpm = rpmMAX;
        }
        // 计算定时器周期
        uint16_t period = (uint16_t)(72000000 / 720 / (rpm / 60 * 360 / stepAngle)) - 1;
        // 计算占空比
        uint16_t duty = (uint16_t)(period / 2);
        // 设置占空比
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
        // 停止TIM1
        HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
        // 设置TIM1周期
        __HAL_TIM_SET_AUTORELOAD(&htim1, period);
        __HAL_TIM_SET_COUNTER(&htim1, 0);
        // 启动TIM1
        HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);

        printf("Set Motor speed to %.2f RPM\n", rpm);
    }
}

void silentcontrol(float rpm)
{
    // rpm为0.6~600，对应定时器周期50000~100
    if (rpm >= 0)
    {
        if (rpm < 0.6)
        {
            // 停止TIM1
            HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
            // printf("Stop\n");
            return;
        }
        if (rpm > rpmMAX)
        {
            rpm = rpmMAX;
        }
        // 计算定时器周期
        uint16_t period = (uint16_t)(72000000 / 720 / (rpm / 60 * 360 / stepAngle)) - 1;
        // 计算占空比
        uint16_t duty = (uint16_t)(period / 2);
        // 设置占空比
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
        // 停止TIM1
        HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
        // 设置TIM1周期
        __HAL_TIM_SET_AUTORELOAD(&htim1, period);
        __HAL_TIM_SET_COUNTER(&htim1, 0);
        // 启动TIM1
        HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);

        // printf("Set Motor speed to %.2f RPM\n", rpm);
    }
}

void forward(void){
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    printf("Forward!\n");
}

void backward(void){
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
    printf("Backward!\n");
}

void enable(void){
    HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_SET);
    printf("Enable motor!\n");
}

void disable(void){
    HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_RESET);
    printf("Disable motor\n");
}
