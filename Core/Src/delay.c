#include "main.h"
#include "stdio.h"

volatile uint64_t msTicks; // 毫秒计数器

void delay_ms(uint32_t ms)
{
    msTicks = 0;
    while(msTicks < ms); // 等待计数器达到指定值
    return;
}
