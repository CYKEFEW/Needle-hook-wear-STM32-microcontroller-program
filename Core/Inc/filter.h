#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>

// 一阶 IIR 低通滤波器（指数平滑）
typedef struct {
    float alpha;      // 0~1，数值越大越平滑（延迟更大）
    float y;
    uint8_t inited;
} lpf1_t;

// 根据截止频率(Hz)和采样率(Hz)配置 alpha。
// 若 cutoff_hz <= 0，则该滤波器旁路直通。
void lpf1_config(lpf1_t *f, float cutoff_hz, float fs_hz);

// 更新滤波输出。若旁路，直接返回 x。
float lpf1_update(lpf1_t *f, float x);

// 死区：抑制小幅振荡/回程间隙噪声。
// 若 |x| < db 则为 0；否则向 0 收缩 db。
float apply_deadband(float x, float db);

// 斜率限制：限制每个控制步的变化量。
float slew_limit(float x, float last, float max_step);

#endif
