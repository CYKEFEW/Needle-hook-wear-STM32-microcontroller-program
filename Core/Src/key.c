#include "main.h"
#include "stdio.h"

extern uint8_t ADCAutoSend;
// HAL库外部中断回调函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if(GPIO_Pin == ZeroKey_Pin){
        printf("ZeroKey pressed\n");
        delay_ms(20);
        if(HAL_GPIO_ReadPin(ZeroKey_GPIO_Port, ZeroKey_Pin) == GPIO_PIN_RESET){
            zero();
        }
    }

    if(GPIO_Pin==sampleKey_Pin){
        printf("sampleKey pressed\n");
        delay_ms(20);
        if(HAL_GPIO_ReadPin(sampleKey_GPIO_Port, sampleKey_Pin) == GPIO_PIN_RESET){
            ADCAutoSend = !ADCAutoSend;
        }
    }
}

