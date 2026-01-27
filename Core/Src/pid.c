#include "main.h"
#include "math.h"
#include "filter.h"

// ========== 可调参数（先给默认值，后续整定时修改） ==========
#define PID_FS_HZ              (50.0f)   // 控制频率/采样频率
#define TENSION_LPF_CUTOFF_HZ  (3.0f)    // 截止频率，≤0 表示不滤波
#define DTERM_LPF_CUTOFF_HZ    (8.0f)    // 微分项滤波截止频率，≤0 表示不滤波
#define ERROR_DEADBAND_N       (0.05f)   // 张力误差死区(N)，0 表示关闭
#define DRPM_MAX_PER_STEP      (16.0f)   // 转速变化率限制(RPM/step)，0 表示关闭

PID Force_Pid = {
    .Kp = 0.1f,    // 待整定
    .Ki = 0.02f,   // 待整定
    .Kd = 0.005f,  // 待整定
    .error = 0.0f,
    .prev_error = 0.0f,
    .prev_prev_error = 0.0f,
    .output = 0.0f,
    .target = 0.0f
};

static lpf1_t g_tbar_lpf;
static lpf1_t g_dterm_lpf;
static uint8_t g_filter_inited = 0u;
static float g_last_rpm = 0.0f;
static float g_prev_dterm_f = 0.0f;

void pid(float CH1, float CH2)
{
    // 平均张力（单位 N）
    const float Tbar = (CH1 + CH2) * 0.5f;

    // 1) 预滤波：抑制装配间隙/偏心等带来的高频噪声
    if (!g_filter_inited) {
        lpf1_config(&g_tbar_lpf, TENSION_LPF_CUTOFF_HZ, PID_FS_HZ);
        lpf1_config(&g_dterm_lpf, DTERM_LPF_CUTOFF_HZ, PID_FS_HZ);
        g_filter_inited = 1u;
    }
    const float Tbar_f = lpf1_update(&g_tbar_lpf, Tbar);

    // 2) 误差（可选死区，抑制微小往复抖动）
    float e = Force_Pid.target - Tbar_f;
    e = apply_deadband(e, ERROR_DEADBAND_N);

    // 3) 微分项滤波：对 (e - e[k-1]) 做一阶低通
    const float dterm_raw = (e - Force_Pid.prev_error);
    const float dterm_f = lpf1_update(&g_dterm_lpf, dterm_raw);

    // 4) 增量式离散 PID（微分项使用滤波后的差分）
    const float du = Force_Pid.Kp * (e - Force_Pid.prev_error)
                   + Force_Pid.Ki * e
                   + Force_Pid.Kd * (dterm_f - g_prev_dterm_f);

    Force_Pid.output += du;

    Force_Pid.prev_prev_error = Force_Pid.prev_error;
    Force_Pid.prev_error = e;
    Force_Pid.error = e;
    g_prev_dterm_f = dterm_f;

    // 5) 输出限幅（防止过大调速导致振荡/超速）
    if (Force_Pid.output > 15.0f) {
        Force_Pid.output = 15.0f;
    } else if (Force_Pid.output < -15.0f) {
        Force_Pid.output = -15.0f;
    }

    // 6) 映射为转速指令（保持你当前“*50”的工程映射）
    float rpm = Force_Pid.output * 50.0f;

    // 速度限幅（与原工程一致，可按实际安全值修改）
    if (rpm > 400.0f) {
        rpm = 400.0f;
    } else if (rpm < 0.0f) {
        rpm = 0.0f;
    }

    // 转速变化率限制：降低高频调速引起的机械振动
    rpm = slew_limit(rpm, g_last_rpm, DRPM_MAX_PER_STEP);
    g_last_rpm = rpm;

    // 目标为 0 时停止，并重置微分滤波状态
    if (fabsf(Force_Pid.target) < 0.01f) {
        rpm = 0.0f;
        g_last_rpm = 0.0f;
        g_prev_dterm_f = 0.0f;
        g_dterm_lpf.inited = 0u;
    }

    HAL_GPIO_TogglePin(PID_LED_GPIO_Port, PID_LED_Pin);

    // 控制电机
    silentcontrol(rpm);
}
