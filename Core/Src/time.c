#include "main.h"
#include "stdio.h"

extern volatile uint64_t msTicks;
extern uint16_t adc_value[DATA_NUM];
extern ADC_HandleTypeDef hadc1;
extern volatile uint8_t g_modbus2_parse_ok; // 串口2接收数据是否完成标志位
extern volatile Modbus2Data g_modbus2_data; // 串口2接收数据缓存区

// TIM2中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2)  // 确认是TIM2中断
    {
        msTicks++; // 每次中断，计数器加1
    } else if (htim->Instance == TIM3) {
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_value, DATA_NUM);
    }

    // if(htim->Instance == TIM6)
    // {
    //     USART2_ModbusPoll();
    //     if(g_modbus2_parse_ok == 1) {
    //         g_modbus2_parse_ok = 0;
    //         printf("{CH1/CH2:%.2f,%.2f}\n", g_modbus2_data.py1, g_modbus2_data.py2);
    //     }
        
    // }
}
