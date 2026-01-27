#include "main.h"
#include "math.h"
#include "filter.h"

// ========== 可调参数（先给默认值，后续整定时修改） ==========
#define PID_FS_HZ              (50.0f)   // 控制频率/采样频率
#define TENSION_LPF_CUTOFF_HZ  (3.0f)    // 截止频率，≤0 表示不滤波
#define DTERM_LPF_CUTOFF_HZ    (8.0f)    // 微分项滤波截止频率，≤0 表示不滤波
#define ERROR_DEADBAND_N       (0.05f)   // 张力误差死区(N)，0 表示关闭
#define DRPM_MAX_PER_STEP      (16.0f)   // 转速变化率限制(RPM/step)，0 表示关闭
#define NOTCH_ENABLE_RPM_MIN   (30.0f)
#define NOTCH_K                (1.0f)    // 传动比*谐波倍数，默认1
#define NOTCH_F0_MIN_HZ        (0.3f)
#define NOTCH_F0_MAX_HZ        (20.0f)

#define NOTCH_F0_BETA          (0.2f)
#define NOTCH_F0_EPS_HZ        (0.05f)

#define NOTCH_BW_HZ            (0.4f)    // 目标绝对带宽Δf
#define NOTCH_Q_MIN            (4.0f)
#define NOTCH_Q_MAX            (20.0f)
#define NOTCH_Q_BETA           (0.2f)
#define NOTCH_Q_EPS            (0.2f)
// ==========================================================

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

// 滤波器状态
static lpf1_t g_tbar_lpf;   // 张力一阶低通滤波器
static lpf1_t g_dterm_lpf;  // 微分项一阶低通滤波器
static notch2_t g_tbar_notch;   // 张力二阶陷波器
static uint8_t g_notch_cfg_inited=0u;   // 陷波器配置状态
static float g_f0_sm=0.0f;  // 陷波器中心频率平滑值
static float g_Q_sm=0.0f;   // 陷波器Q值平滑值
static uint8_t g_filter_inited = 0u;    // 滤波器初始化状态
static float g_last_rpm = 0.0f; // 上次控制步的转速
static float g_prev_dterm_f = 0.0f; // 上次控制步的微分项滤波值

void pid(float CH1, float CH2)
{
    // 平均张力（单位 N）
    const float Tbar = (CH1 + CH2) * 0.5f;
    float Tbar_pre = Tbar;
    // 用上一周期输出的转速(此时g_last_rpm还是上一次的值)
    const float rpm_for_notch = g_last_rpm;

    // 1) 预滤波：抑制装配间隙/偏心等带来的高频噪声
    if(rpm_for_notch >= NOTCH_ENABLE_RPM_MIN){
        float f0_target = (rpm_for_notch/60.0f) * NOTCH_K;
        if(f0_target<NOTCH_F0_MIN_HZ)f0_target = NOTCH_F0_MIN_HZ;
        if(f0_target>NOTCH_F0_MAX_HZ)f0_target = NOTCH_F0_MAX_HZ;

        // 平滑f0，避免系数抖动
        if(!g_notch_cfg_inited){
            g_f0_sm = f0_target;
        } else {
            g_f0_sm=g_f0_sm + NOTCH_F0_BETA * (f0_target-g_f0_sm);
        }

        // 动态Q：保持绝对带宽Δf≈NOTCH_BW_HZ
        float Q_target = g_f0_sm / NOTCH_BW_HZ;
        if(Q_target<NOTCH_Q_MIN)Q_target = NOTCH_Q_MIN;
        if(Q_target>NOTCH_Q_MAX)Q_target = NOTCH_Q_MAX;

        if(!g_notch_cfg_inited){
            g_Q_sm = Q_target;
            notch2_config(&g_tbar_notch,g_f0_sm,g_Q_sm,PID_FS_HZ);
            g_tbar_notch.inited= 0u ;
            g_notch_cfg_inited= 1u ;
        } else {
            g_Q_sm=g_Q_sm+NOTCH_Q_BETA*(Q_target-g_Q_sm);
            // f0或Q变化明显才重算系数
            if(fabsf(g_f0_sm-g_tbar_notch.f0)>NOTCH_F0_EPS_HZ ||
            fabsf(g_Q_sm-g_tbar_notch.Q)>NOTCH_Q_EPS){
                notch2_config(&g_tbar_notch,g_f0_sm,g_Q_sm,PID_FS_HZ);
            }
        }

        Tbar_pre=notch2_update(&g_tbar_notch,Tbar);
    } else {
        // 低速旁路
        g_tbar_notch.inited=0u;
        g_notch_cfg_inited=0u;
    }

    if (!g_filter_inited) {
        lpf1_config(&g_tbar_lpf, TENSION_LPF_CUTOFF_HZ, PID_FS_HZ);
        lpf1_config(&g_dterm_lpf, DTERM_LPF_CUTOFF_HZ, PID_FS_HZ);
        g_filter_inited = 1u;
    }
    const float Tbar_f = lpf1_update(&g_tbar_lpf, Tbar_pre);

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
        g_tbar_notch.inited = 0u;
        g_notch_cfg_inited = 0u;
    }

    HAL_GPIO_TogglePin(PID_LED_GPIO_Port, PID_LED_Pin);

    // 控制电机
    silentcontrol(rpm);
}
