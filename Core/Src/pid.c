#include "main.h"
#include "filter.h"

#include <math.h>

// 可调控制参数。
#define PID_FS_HZ               (50.0f)
#define TENSION_LPF_CUTOFF_HZ   (3.0f)
#define DTERM_LPF_CUTOFF_HZ     (8.0f)
#define ERROR_DEADBAND_N        (0.05f)
#define DRPM_MAX_PER_STEP       (16.0f)
#define MOTOR_MIN_EFFECTIVE_RPM (0.6f)
#define MOTOR_RESTART_RPM       (1.0f)
#define MOTOR_MAX_RPM           (400.0f)

#define NOTCH_ENABLE_RPM_MIN    (30.0f)
#define NOTCH_K                 (1.0f)
#define NOTCH_F0_MIN_HZ         (0.3f)
#define NOTCH_F0_MAX_HZ         (20.0f)
#define NOTCH_F0_BETA           (0.2f)
#define NOTCH_F0_EPS_HZ         (0.05f)
#define NOTCH_BW_HZ             (0.4f)
#define NOTCH_Q_MIN             (4.0f)
#define NOTCH_Q_MAX             (20.0f)
#define NOTCH_Q_BETA            (0.2f)
#define NOTCH_Q_EPS             (0.2f)

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
static float g_last_applied_rpm = 0.0f;
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

void pid(float CH1, float CH2, float rpm)
{
    const float Tbar = (CH1 + CH2) * 0.5f;
    float Tbar_pre = Tbar;
    const float rpm_for_notch = g_last_applied_rpm;

    if (rpm_for_notch >= NOTCH_ENABLE_RPM_MIN) {
        float f0_target = (rpm_for_notch / 60.0f) * NOTCH_K;
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

    const float Tbar_f = lpf1_update(&g_tbar_lpf, Tbar_pre);

    float e = Force_Pid.target - Tbar_f;
    e = apply_deadband(e, ERROR_DEADBAND_N);

    const float dterm_raw = e - Force_Pid.prev_error;
    const float dterm_f = lpf1_update(&g_dterm_lpf, dterm_raw);

    const float du = Force_Pid.Kp * (e - Force_Pid.prev_error) +
                     Force_Pid.Ki * e +
                     Force_Pid.Kd * (dterm_f - g_prev_dterm_f);

    Force_Pid.output += du;
    Force_Pid.prev_prev_error = Force_Pid.prev_error;
    Force_Pid.prev_error = e;
    Force_Pid.error = e;
    g_prev_dterm_f = dterm_f;

    const float pid_out_max = MOTOR_MAX_RPM / 50.0f;
    Force_Pid.output = clampf_local(Force_Pid.output, 0.0f, pid_out_max);

    const float rpm_target = clampf_local(Force_Pid.output * 50.0f, 0.0f, MOTOR_MAX_RPM);
    float rpm_cmd = slew_limit(rpm_target, g_last_applied_rpm, DRPM_MAX_PER_STEP);

    // 控制器内部状态不受执行器死区影响，避免输出被停转逻辑反向拉低。
    if (rpm_cmd > 0.0f && rpm_cmd < MOTOR_MIN_EFFECTIVE_RPM) {
        if (g_last_applied_rpm < MOTOR_MIN_EFFECTIVE_RPM &&
            e > 0.0f &&
            rpm_target >= MOTOR_MIN_EFFECTIVE_RPM) {
            rpm_cmd = MOTOR_RESTART_RPM;
            if (rpm_cmd < MOTOR_MIN_EFFECTIVE_RPM) {
                rpm_cmd = MOTOR_MIN_EFFECTIVE_RPM;
            }
        } else {
            rpm_cmd = 0.0f;
        }
    }

    g_last_applied_rpm = rpm_cmd;

    if (fabsf(Force_Pid.target) < 0.01f) {
        rpm_cmd = 0.0f;
        g_last_applied_rpm = 0.0f;
        Force_Pid.output = 0.0f;
        Force_Pid.error = 0.0f;
        Force_Pid.prev_error = 0.0f;
        Force_Pid.prev_prev_error = 0.0f;
        g_prev_dterm_f = 0.0f;
        g_dterm_lpf.inited = 0u;
        g_tbar_notch.inited = 0u;
        g_notch_cfg_inited = 0u;
    }

    HAL_GPIO_TogglePin(PID_LED_GPIO_Port, PID_LED_Pin);
    silentcontrol(rpm_cmd);
}
