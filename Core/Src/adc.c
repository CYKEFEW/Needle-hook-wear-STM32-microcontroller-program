#include "main.h"

// ADC初始化函数
extern ADC_HandleTypeDef hadc1;
extern uint16_t adc_value[2];

// ADC回调函数
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc == &hadc1) {  // 确保是正确的ADC实例
        for (int i = 0; i < 2; i++) {
            adc_value[i] = HAL_ADC_GetValue(&hadc1);  // 获取ADC转换结果
        }
        // HAL_ADC_Start_IT(&hadc1);  // 启动下一次ADC转换
    }
}
