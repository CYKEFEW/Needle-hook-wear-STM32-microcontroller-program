#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 一阶 IIR 低通滤波器（指数平滑）
typedef struct {
    float alpha;     // alpha=exp(-2*pi*fc/fs)，<=0 表示旁路
    float y;
    uint8_t inited;
} lpf1_t;

void  lpf1_config(lpf1_t *f, float cutoff_hz, float fs_hz);
float lpf1_update(lpf1_t *f, float x);

// 二阶 IIR 陷波（notch）滤波器（biquad）
typedef struct{
    float b0,b1,b2,a1,a2;
    float x1,x2,y1,y2;
    float f0,fs,Q;
    uint8_t inited;
} notch2_t;

void  notch2_config(notch2_t *f, float f0_hz, float Q, float fs_hz);
float notch2_update(notch2_t *f, float x);

// 误差死区：|x|<db 输出0，否则向0收缩db
float apply_deadband(float x, float db);

// 斜率限制：限制每个控制步的变化量
float slew_limit(float x, float last, float max_step);

#ifdef __cplusplus
}
#endif

#endif
