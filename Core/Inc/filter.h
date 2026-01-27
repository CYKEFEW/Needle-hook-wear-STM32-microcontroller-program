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

// 二阶 IIR 带阻滤波器（陷波器）
typedef struct{
    float b0,b1,b2,a1,a2;
    float x1,x2,y1,y2;
    float f0,fs,Q;
    uint8_t inited;
} notch2_t;

// 配置二阶陷波器参数。
void notch2_config(notch2_t *f,float f0_hz,float Q,float fs_hz);
// 更新二阶陷波器输出。
float notch2_update(notch2_t *f,float x);

// 死区：抑制小幅振荡/回程间隙噪声。
// 若 |x| < db 则为 0；否则向 0 收缩 db。
float apply_deadband(float x, float db);

// 斜率限制：限制每个控制步的变化量。
float slew_limit(float x, float last, float max_step);

#endif
