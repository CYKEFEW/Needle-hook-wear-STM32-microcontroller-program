#include "main.h"
#include "stdio.h"

extern uint16_t adc_value[DATA_NUM];
float zero_voltage[DATA_NUM] = {0.0f,0.0f};
float voltage[DATA_NUM] = {0.0f,0.0f};

void Senddata(){
    voltage[0] = (float)adc_value[0] / 4095.0 * 3.3f;
    voltage[1] = (float)adc_value[1] / 4095.0 * 3.3f;
    printf("{F/g:%.2f,%.2f}\n",(voltage[0] - zero_voltage[0])*872.5f/2.52f,voltage[1] - zero_voltage[1]);
}
