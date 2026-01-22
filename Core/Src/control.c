#include "main.h"
#include "stdio.h"

float stepAngle = 0.225f;
uint16_t rpmMAX = 1500;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
void control(float rpm)
{
    // rpm为0.6~600，对应定时器周期50000~100
    if (rpm >= 0)
    {
        if (rpm < 0.6)
        {
            // 停止TIM1
            HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
            printf("Stop\n");
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

void control2(float rpm)
{
    // rpm为0.6~600，对应定时器周期50000~100
    if (rpm >= 0)
    {
        if (rpm < 0.6)
        {
            // 停止TIM4
            HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);
            printf("Stop\n");
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
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, duty);
        // 停止TIM4
        HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);
        // 设置TIM4周期
        __HAL_TIM_SET_AUTORELOAD(&htim4, period);
        __HAL_TIM_SET_COUNTER(&htim4, 0);
        // 启动TIM4
        HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);

        printf("Set Motor speed to %.2f RPM\n", rpm);
    }
}

void forward(void){
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    printf("Forward!\n");
}

void forward2(void){
    HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_SET);
    printf("Forward2!\n");
}

void backward(void){
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
    printf("Backward!\n");
}

void backward2(void){
    HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_RESET);
    printf("Backward2!\n");
}

void enable(void){
    HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_SET);
    printf("Enable motor!\n");
}

void enable2(void){
    HAL_GPIO_WritePin(ENA2_GPIO_Port, ENA2_Pin, GPIO_PIN_SET);
    printf("Enable motor2!\n");
}

void disable(void){
    HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_RESET);
    printf("Disable motor\n");
}

/**
 * @brief 禁用电机2的函数
 * 该函数通过将使能引脚设置为低电平来禁用电机2，
 * 并打印一条禁用电机的信息到控制台
 */
void disable2(void){
    HAL_GPIO_WritePin(ENA2_GPIO_Port, ENA2_Pin, GPIO_PIN_RESET); // 将电机2的使能引脚设置为低电平，禁用电机
    printf("Disable motor2\n");
}
