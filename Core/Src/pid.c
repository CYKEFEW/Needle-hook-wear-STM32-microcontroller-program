#include "main.h"
#include "filter.h"

#include <math.h>

// 可调控制参数。
#define PID_FS_HZ             (50.0f)  // PID 控制频率，单位 Hz
#define TENSION_LPF_CUTOFF_HZ (3.0f)   // 张力均值低通截止频率，单位 Hz
#define DTERM_LPF_CUTOFF_HZ   (8.0f)   // 微分项低通截止频率，单位 Hz
#define ERROR_DEADBAND_N      (0.05f)  // 张力误差死区，单位 N
#define TORQUE_OUTPUT_MIN_NM  (0.0f)   // 转矩输出下限，单位 Nm
#define TORQUE_OUTPUT_MAX_NM  (1.0f)   // 转矩输出上限，单位 Nm
#define MOTOR_MAX_RPM         (400.0f) // silentcontrol 输出转速上限，单位 RPM

#define NOTCH_ENABLE_RPM_MIN (30.0f) // 启用陷波的最小目标转速，单位 RPM
#define NOTCH_K              (1.0f)  // 目标转速映射到陷波中心频率的比例系数
#define NOTCH_F0_MIN_HZ      (0.3f)  // 陷波中心频率下限，单位 Hz
#define NOTCH_F0_MAX_HZ      (20.0f) // 陷波中心频率上限，单位 Hz
#define NOTCH_F0_BETA        (0.2f)  // 陷波中心频率平滑系数
#define NOTCH_F0_EPS_HZ      (0.05f) // 陷波中心频率重配置阈值，单位 Hz
#define NOTCH_BW_HZ          (0.4f)  // 陷波带宽，单位 Hz
#define NOTCH_Q_MIN          (4.0f)  // 陷波 Q 值下限
#define NOTCH_Q_MAX          (20.0f) // 陷波 Q 值上限
#define NOTCH_Q_BETA         (0.2f)  // 陷波 Q 值平滑系数
#define NOTCH_Q_EPS          (0.2f)  // 陷波 Q 值重配置阈值

PID Force_Pid = {
    .Kp = 0.3f,
    .Ki = 0.1f,
    .Kd = 0.02f,
    .error = 0.0f,
    .prev_error = 0.0f,
    .prev_prev_error = 0.0f,
    .output = 0.0f,
    .target = 0.0f,
};

static lpf1_t g_tbar_lpf;
static lpf1_t g_dterm_lpf;
static notch2_t g_tbar_notch;
static uint8_t g_notch_cfg_inited = 0u;
static float g_f0_sm = 0.0f;
static float g_Q_sm = 0.0f;
static uint8_t g_filter_inited = 0u;
static float g_prev_dterm_f = 0.0f;

static float clampf_local(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static void reset_pid_state(void)
{
    Force_Pid.error = 0.0f;
    Force_Pid.prev_error = 0.0f;
    Force_Pid.prev_prev_error = 0.0f;
    Force_Pid.output = TORQUE_OUTPUT_MIN_NM;
    g_prev_dterm_f = 0.0f;

    g_tbar_lpf.inited = 0u;
    g_dterm_lpf.inited = 0u;
    g_tbar_notch.inited = 0u;
    g_filter_inited = 0u;
    g_notch_cfg_inited = 0u;
    g_f0_sm = 0.0f;
    g_Q_sm = 0.0f;
}

void pid(float CH1, float CH2, float target_rpm)
{
    const float Tbar = (CH1 + CH2) * 0.5f;
    float Tbar_pre = Tbar;
    const float rpm_cmd = clampf_local(target_rpm, 0.0f, MOTOR_MAX_RPM);

    if (rpm_cmd >= NOTCH_ENABLE_RPM_MIN) {
        float f0_target = (rpm_cmd / 60.0f) * NOTCH_K;
        f0_target = clampf_local(f0_target, NOTCH_F0_MIN_HZ, NOTCH_F0_MAX_HZ);

        if (!g_notch_cfg_inited) {
            g_f0_sm = f0_target;
        } else {
            g_f0_sm += NOTCH_F0_BETA * (f0_target - g_f0_sm);
        }

        float Q_target = g_f0_sm / NOTCH_BW_HZ;
        Q_target = clampf_local(Q_target, NOTCH_Q_MIN, NOTCH_Q_MAX);

        if (!g_notch_cfg_inited) {
            g_Q_sm = Q_target;
            notch2_config(&g_tbar_notch, g_f0_sm, g_Q_sm, PID_FS_HZ);
            g_tbar_notch.inited = 0u;
            g_notch_cfg_inited = 1u;
        } else {
            g_Q_sm += NOTCH_Q_BETA * (Q_target - g_Q_sm);
            if (fabsf(g_f0_sm - g_tbar_notch.f0) > NOTCH_F0_EPS_HZ ||
                fabsf(g_Q_sm - g_tbar_notch.Q) > NOTCH_Q_EPS) {
                notch2_config(&g_tbar_notch, g_f0_sm, g_Q_sm, PID_FS_HZ);
            }
        }

        Tbar_pre = notch2_update(&g_tbar_notch, Tbar);
    } else {
        g_tbar_notch.inited = 0u;
        g_notch_cfg_inited = 0u;
    }

    if (!g_filter_inited) {
        lpf1_config(&g_tbar_lpf, TENSION_LPF_CUTOFF_HZ, PID_FS_HZ);
        lpf1_config(&g_dterm_lpf, DTERM_LPF_CUTOFF_HZ, PID_FS_HZ);
        g_filter_inited = 1u;
    }

    if (fabsf(Force_Pid.target) < 0.01f) {
        reset_pid_state();
        HAL_GPIO_TogglePin(PID_LED_GPIO_Port, PID_LED_Pin);
        setTorque(Force_Pid.output);
        silentcontrol(rpm_cmd);
        return;
    }

    {
        const float Tbar_f = lpf1_update(&g_tbar_lpf, Tbar_pre);
        float e = Force_Pid.target - Tbar_f;

        e = apply_deadband(e, ERROR_DEADBAND_N);

        {
            const float dterm_raw = e - Force_Pid.prev_error;
            const float dterm_f = lpf1_update(&g_dterm_lpf, dterm_raw);

            Force_Pid.output += Force_Pid.Kp * (e - Force_Pid.prev_error) +
                                Force_Pid.Ki * e +
                                Force_Pid.Kd * (dterm_f - g_prev_dterm_f);
            Force_Pid.prev_prev_error = Force_Pid.prev_error;
            Force_Pid.prev_error = e;
            Force_Pid.error = e;
            g_prev_dterm_f = dterm_f;
        }
    }

    Force_Pid.output = clampf_local(Force_Pid.output, TORQUE_OUTPUT_MIN_NM, TORQUE_OUTPUT_MAX_NM);

    HAL_GPIO_TogglePin(PID_LED_GPIO_Port, PID_LED_Pin);
    setTorque(Force_Pid.output);
    silentcontrol(rpm_cmd);
}
