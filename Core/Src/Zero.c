#include "main.h"
#include "stdio.h"

extern uint16_t adc_value[2];
extern float zero_voltage[2];
extern float voltage[2];

void zero(void){
    voltage[0] = (float)adc_value[0] / 4095.0 * 3.3f; // 读取电压值
    voltage[1] = (float)adc_value[1] / 4095.0 * 3.3f;
    erase_flash();
    float tempData[2] = {voltage[0],voltage[1]};
    write_flash(tempData);
    zero_voltage[0] = voltage[0];
    zero_voltage[1] = voltage[1];
    printf("ZeroVoltage0 set to %f V\n",voltage[0]);
    printf("ZeroVoltage1 set to %f V\n",voltage[1]);
}

