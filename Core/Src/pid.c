#include "main.h"
#include "filter.h"

#include <math.h>

// 可调控制参数。
#define PID_FS_HZ             (50.0f)  // PID 控制频率，单位 Hz
#define TENSION_LPF_CUTOFF_HZ (3.0f)   // 张力均值低通截止频率，单位 Hz
#define DTERM_LPF_CUTOFF_HZ   (8.0f)   // 微分项低通截止频率，单位 Hz
#define ERROR_DEADBAND_N      (0.02f)  // 张力误差死区，单位 N
#define TORQUE_OUTPUT_MIN_NM  (0.0f)   // 力矩输出下限，单位 Nm
#define TORQUE_OUTPUT_MAX_NM  (0.05f)   // 力矩输出上限，单位 Nm
#define MOTOR_MAX_RPM         (400.0f) // silentcontrol 输出转速上限，单位 RPM
#define PID_INCREMENT_RATIO   (0.05f)   // 1 N 满量程误差对应 0.05 Nm 满量程力矩输出

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

// 张力闭环，PID 输出为电机力矩命令。
PID Force_Pid = {
    .Kp = 0.30f,
    .Ki = 0.018f,
    .Kd = 0.04f,
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
    // 将两个通道的张力值取平均，作为当前控制环使用的输入张力。
    const float Tbar = (CH1 + CH2) * 0.5f;
    // Tbar_pre 表示送入后级滤波/PID 前的张力值，默认先使用原始平均值。
    float Tbar_pre = Tbar;
    // 对目标转速做限幅，避免给电机发送超出允许范围的转速命令。
    const float rpm_cmd = clampf_local(target_rpm, 0.0f, MOTOR_MAX_RPM);

    // 目标转速较高时启用陷波滤波，抑制与转速相关的周期性干扰。
    if (rpm_cmd >= NOTCH_ENABLE_RPM_MIN) {
        // 将目标转速映射为陷波中心频率，并限制在配置允许范围内。
        float f0_target = (rpm_cmd / 60.0f) * NOTCH_K;
        f0_target = clampf_local(f0_target, NOTCH_F0_MIN_HZ, NOTCH_F0_MAX_HZ);

        // 对中心频率做平滑，避免转速变化时滤波器参数突变。
        if (!g_notch_cfg_inited) {
            g_f0_sm = f0_target;
        } else {
            g_f0_sm += NOTCH_F0_BETA * (f0_target - g_f0_sm);
        }

        // 根据中心频率和带宽计算 Q 值，并同样做限幅和平滑。
        float Q_target = g_f0_sm / NOTCH_BW_HZ;
        Q_target = clampf_local(Q_target, NOTCH_Q_MIN, NOTCH_Q_MAX);

        // 首次进入时直接完成陷波器配置；后续仅在参数变化足够大时重配置。
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

        // 将平均张力送入陷波器，得到抑制干扰后的张力信号。
        Tbar_pre = notch2_update(&g_tbar_notch, Tbar);
    } else {
        // 低速时不使用陷波器，并清掉配置状态，便于下次重新初始化。
        g_tbar_notch.inited = 0u;
        g_notch_cfg_inited = 0u;
    }

    // 首次进入时初始化张力低通和微分项低通滤波器。
    if (!g_filter_inited) {
        lpf1_config(&g_tbar_lpf, TENSION_LPF_CUTOFF_HZ, PID_FS_HZ);
        lpf1_config(&g_dterm_lpf, DTERM_LPF_CUTOFF_HZ, PID_FS_HZ);
        g_filter_inited = 1u;
    }

    // 若目标张力接近 0，认为闭环暂不工作，重置 PID 与滤波器状态并直接输出最小力矩。
    if (fabsf(Force_Pid.target) < 0.002f) {
        reset_pid_state();
        HAL_GPIO_TogglePin(PID_LED_GPIO_Port, PID_LED_Pin);
        setTorque(Force_Pid.output);
        silentcontrol(rpm_cmd);
        return;
    }

    {
        // 对张力信号做低通，减少测量噪声对误差和控制输出的影响。
        const float Tbar_f = lpf1_update(&g_tbar_lpf, Tbar_pre);
        // 当前误差 = 目标张力 - 实际张力。
        float e = Force_Pid.target - Tbar_f;

        // 小误差直接压到死区内，减少输出抖动。
        e = apply_deadband(e, ERROR_DEADBAND_N);

        {
            // 误差变化量作为微分原始输入，再经过低通降低微分放大噪声的问题。
            const float dterm_raw = e - Force_Pid.prev_error;
            const float dterm_f = lpf1_update(&g_dterm_lpf, dterm_raw);

            // 使用增量式 PID 更新输出：
            // 比例项看当前误差变化，积分项累积当前误差，微分项看滤波后的误差变化率。
            Force_Pid.output += PID_INCREMENT_RATIO * (Force_Pid.Kp * (e - Force_Pid.prev_error) +
                                                       Force_Pid.Ki * e +
                                                       Force_Pid.Kd * (dterm_f - g_prev_dterm_f));
            // 保存历史误差和微分状态，供下一次控制周期使用。
            Force_Pid.prev_prev_error = Force_Pid.prev_error;
            Force_Pid.prev_error = e;
            Force_Pid.error = e;
            g_prev_dterm_f = dterm_f;
        }
    }

    // 对输出力矩限幅，确保命令始终在电机允许范围内。
    Force_Pid.output = clampf_local(Force_Pid.output, TORQUE_OUTPUT_MIN_NM, TORQUE_OUTPUT_MAX_NM);

    // 输出本次 PID 计算结果，同时下发电机转速命令。
    HAL_GPIO_TogglePin(PID_LED_GPIO_Port, PID_LED_Pin);
    setTorque(Force_Pid.output);
    silentcontrol(rpm_cmd);
}
