#include "main.h"

// PID调度缓存：由USART1接收到Data命令后写入，TIM2中断读取。
extern volatile float g_pid_ch1;
// PID调度缓存：由USART1接收到Data命令后写入，TIM2中断读取。
extern volatile float g_pid_ch2;
// PID调度缓存：由USART1接收到Data命令后写入，TIM2中断读取。
extern volatile float g_pid_target_rpm;
// PID调度标志：为1表示TIM2下一次溢出时需要执行一次PID计算。
extern volatile uint8_t g_pid_pending;

// TIM2溢出回调：扫描PID调度标志，若有新的张力数据则执行一次PID计算。
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        if (g_pid_pending)
        {
            // 先把全局缓存复制到局部变量，再清除标志，避免PID执行期间新数据被误清除。
            const float CH1 = g_pid_ch1;
            const float CH2 = g_pid_ch2;
            const float target_rpm = g_pid_target_rpm;
            g_pid_pending = 0u;

            pid(CH1, CH2, target_rpm);
        }
    }
}
