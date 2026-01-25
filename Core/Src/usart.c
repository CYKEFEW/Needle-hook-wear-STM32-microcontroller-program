#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern uint16_t adc_value;
extern uint8_t ADCAutoSend;
extern float voltage;
extern float zero_voltage;

/* 这些函数在你工程其他文件里实现（这里仅声明，避免编译告警） */
extern void Senddata(void);
extern void control(float value);
extern void zero(void);
extern void forward(void);
extern void backward(void);
extern void enable(void);
extern void disable(void);
extern void forward2(void);
extern void backward2(void);
extern void enable2(void);
extern void disable2(void);

/* =========================
 * USART1: 终端命令接收
 * ========================= */
static uint8_t usart1_cmd_buf[512];
static uint8_t usart1_cmd_idx = 0;
volatile char usart1_rx_byte = 0;

/* =========================
 * USART2: Modbus RTU(显示仪 -> 单片机)
 * 根据你给的格式(表7/表8)：
 *   请求: [Addr][03][00][00][00][02][CRCLo][CRCHi]
 *   响应: [Addr][03][06][PY1Hi][PY1Lo][PY2Hi][PY2Lo][DP1][DP2][CRCLo][CRCHi]
 *   - PY1/PY2: 16bit 有符号整形(高字节在前)
 *   - DP1/DP2: 小数点位置(小数位数)，例如 DP=2 表示除以100
 *
 * 接收策略：USART2 逐字节中断接收 + 帧间超时判帧
 * 使用方法：
 *   1) 初始化后调用一次 USART2_ModbusInit();
 *   2) 在 while(1) 或定时任务里周期调用 USART2_ModbusPoll();
 *      (建议 1ms~10ms 调一次)
 *
 * 解析成功标志：g_modbus2_parse_ok (成功=1 失败=0)
 * 解析结果：g_modbus2_data.py1 / g_modbus2_data.py2
 * ========================= */

#define MODBUS2_RX_BUF_SIZE          64u
#define MODBUS2_FRAME_TIMEOUT_MS     5u   /* 帧间隔超时(单位ms)，根据波特率可调(建议3~10ms) */

volatile uint8_t g_modbus2_parse_ok = 0; /* 你要的“成功示1失败示0” */
volatile Modbus2Data g_modbus2_data;       /* 解析后的数据 */

volatile uint8_t s_u2_rx_byte = 0;
static uint8_t           s_u2_frame[MODBUS2_RX_BUF_SIZE];
static volatile uint16_t s_u2_len = 0;
static volatile uint32_t s_u2_last_tick = 0;

static uint16_t Modbus_CRC16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= buf[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

static float pow10_u8(uint8_t n)
{
    /* 避免引入 math 库：n 通常很小(0~4) */
    static const float p10[] = {1.0f, 10.0f, 100.0f, 1000.0f, 10000.0f, 100000.0f, 1000000.0f};
    if (n < (uint8_t)(sizeof(p10) / sizeof(p10[0])))
        return p10[n];
    return p10[(sizeof(p10) / sizeof(p10[0])) - 1];
}

static uint8_t Modbus2_ParseFrame(const uint8_t *frame, uint16_t len)
{
    /* 期望长度 = 11: addr(1)+func(1)+bytecnt(1)+data(6)+crc(2) */
    if (len != 11u) return 0;

    uint16_t crc_calc = Modbus_CRC16(frame, (uint16_t)(len - 2u));
    uint16_t crc_recv = (uint16_t)frame[len - 2u] | ((uint16_t)frame[len - 1u] << 8);
    if (crc_calc != crc_recv) return 0;

    uint8_t addr = frame[0];
    uint8_t func = frame[1];

    if (func & 0x80u) return 0;      /* 异常响应 */
    if (func != 0x03u) return 0;
    if (frame[2] != 0x06u) return 0; /* 数据长度必须是 6 */

    int16_t py1 = (int16_t)((uint16_t)frame[3] << 8 | frame[4]);
    int16_t py2 = (int16_t)((uint16_t)frame[5] << 8 | frame[6]);
    uint8_t dp1 = frame[7];
    uint8_t dp2 = frame[8];

    g_modbus2_data.ok = 1;
    g_modbus2_data.addr = addr;
    g_modbus2_data.py1_raw = py1;
    g_modbus2_data.py2_raw = py2;
    g_modbus2_data.dp1 = dp1;
    g_modbus2_data.dp2 = dp2;
    g_modbus2_data.py1 = ((float)py1) / pow10_u8(dp1);
    g_modbus2_data.py2 = ((float)py2) / pow10_u8(dp2);

    return 1;
}

void USART2_ModbusInit(void)
{
    s_u2_len = 0;
    s_u2_last_tick = HAL_GetTick();
    g_modbus2_parse_ok = 0;
    memset((void *)&g_modbus2_data, 0, sizeof(g_modbus2_data));

    /* 启动 USART2 单字节中断接收 */
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&s_u2_rx_byte, 1);
}

void USART2_ModbusPoll(void)
{
    /* 在主循环/定时任务里调用：如果超过帧间隔超时，则认为一帧接收完成 */
    uint16_t len = s_u2_len;
    if (len == 0u) return;

    uint32_t now = HAL_GetTick();
    if ((now - s_u2_last_tick) < MODBUS2_FRAME_TIMEOUT_MS) return;

    /* 复制到局部缓冲区解析(避免中断期间被改) */
    uint8_t local[MODBUS2_RX_BUF_SIZE];
    if (len > MODBUS2_RX_BUF_SIZE) len = MODBUS2_RX_BUF_SIZE;
    memcpy(local, s_u2_frame, len);

    /* 复位接收长度，准备收下一帧 */
    s_u2_len = 0;

    /* 解析并更新标志 */
    g_modbus2_parse_ok = Modbus2_ParseFrame(local, len) ? 1u : 0u;
}

/* =========================
 * HAL 回调函数
 * ========================= */
volatile uint8_t ConMode = 0; // 0:张力控制模式 1:速度控制模式
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* USART1：命令行 */
    if (huart == &huart1)
    {
        if (usart1_rx_byte == '\n')
        {
            usart1_cmd_buf[usart1_cmd_idx] = '\0';

            if (strncmp((char *)usart1_cmd_buf, "ADC", 3) == 0)
            {
                Senddata();
            }
            else if (strncmp((char *)usart1_cmd_buf, "Auto", 4) == 0)
            {
                ADCAutoSend = 1;
                printf("Auto send ADC value\n");
            }
            else if (strncmp((char *)usart1_cmd_buf, "Stop", 4) == 0)
            {
                ADCAutoSend = 0;
                printf("Stop auto send ADC value\n");
            }
            else if (strncmp((char *)usart1_cmd_buf, "Con ", 4) == 0)
            {
                if (ConMode == 1){
                    float value = atof((char *)usart1_cmd_buf + 4);
                    control(value);
                } else {
                    printf("Please set ConMode to 1 (Speed Control Mode) before using Con command.\n");
                }
            }
            else if (strncmp((char *)usart1_cmd_buf, "Zero", 4) == 0)
            {
                zero();
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
            else if (strncmp((char *)usart1_cmd_buf, "Con2 ", 5) == 0)
            {
                if (ConMode == 1){
                    float value = atof((char *)usart1_cmd_buf + 5);
                    control2(value);
                } else {
                    printf("Please set ConMode to 1 (Speed Control Mode) before using Con2 command.\n");
                }
            }
            else if (strncmp((char *)usart1_cmd_buf, "Forward2", 8) == 0)
            {
                forward2();
            }
            else if (strncmp((char *)usart1_cmd_buf, "Backward2", 9) == 0)
            {
                backward2();
            }
            else if (strncmp((char *)usart1_cmd_buf, "Enable2", 7) == 0)
            {
                enable2();
            }
            else if (strncmp((char *)usart1_cmd_buf, "Disable2", 8) == 0)
            {
                disable2();
            }
            else if (strncmp((char *)usart1_cmd_buf, "Data", 4) == 0){
                float CH1,CH2;
                sscanf((char *)usart1_cmd_buf, "Data %f %f",&CH1,&CH2);
                // printf("CH1:%.2f CH2:%.2f\n",CH1,CH2);
                pid(CH1,CH2);
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
    }
    else if (huart == &huart2)
    {
        s_u2_len = 0;
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&s_u2_rx_byte, 1);
    }

    printf("UART error!\n");
}
