#include "filter.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 一阶 IIR 低通滤波器（指数平滑）
void lpf1_config(lpf1_t *f, float cutoff_hz, float fs_hz)
{
    if (!f) return;
    f->y = 0.0f;
    f->inited = 0u;

    if (cutoff_hz <= 0.0f || fs_hz <= 0.0f) {
        f->alpha = 0.0f; // 旁路标记
        return;
    }

    const float Ts = 1.0f / fs_hz;
    // 系数计算公式：alpha = exp(-2*pi*fc*Ts)
    f->alpha = expf(-2.0f * M_PI * cutoff_hz * Ts);
}

// 更新滤波输出。若旁路，直接返回 x。
float lpf1_update(lpf1_t *f, float x)
{
    if (!f) return x;
    // 旁路
    if (f->alpha <= 0.0f) return x;

    if (!f->inited) {
        f->y = x;
        f->inited = 1u;
        return f->y;
    }

    f->y = f->alpha * f->y + (1.0f - f->alpha) * x;
    return f->y;
}

// 二阶 IIR 带阻滤波器（陷波器）
void notch2_config(notch2_t*f,float f0_hz,float Q,float fs_hz)
{
    if(!f)return;

    f->fs=fs_hz;
    f->Q=Q;
    f->f0=f0_hz;

    // 默认直通
    f->b0=1.0f; f->b1=0.0f; f->b2=0.0f;
    f->a1=0.0f; f->a2=0.0f;

    // 参数不合法则旁路
    if(fs_hz<=0.0f||Q<=0.0f)return;
    if(f0_hz<=0.0f||f0_hz>=0.5f*fs_hz)return;

    const float w0=2.0f*M_PI*f0_hz/fs_hz;
    const float cw=cosf(w0);
    const float sw=sinf(w0);
    const float alpha=sw/(2.0f*Q);

    // 标准二阶陷波(带宽由Q控制)，直流/远离f0处增益约为1
    float b0=1.0f;
    float b1=-2.0f*cw;
    float b2=1.0f;
    float a0=1.0f+alpha;
    float a1=-2.0f*cw;
    float a2=1.0f-alpha;

    f->b0=b0/a0;
    f->b1=b1/a0;
    f->b2=b2/a0;
    f->a1=a1/a0;
    f->a2=a2/a0;
}

// 更新二阶陷波器输出。
float notch2_update(notch2_t*f,float x)
{
    if(!f)return x;

    if(!f->inited){
        f->x1=f->x2=x;
        f->y1=f->y2=x;
        f->inited=1u;
        return x;
    }

    const float y=f->b0*x+f->b1*f->x1+f->b2*f->x2
                -f->a1*f->y1-f->a2*f->y2;

    f->x2=f->x1; f->x1=x;
    f->y2=f->y1; f->y1=y;
    return y;
}

// 死区：抑制小幅振荡/回程间隙噪声。若 |x| < db 则为 0；否则向 0 收缩
float apply_deadband(float x, float db)
{
    if (db <= 0.0f) return x;
    const float ax = fabsf(x);
    if (ax < db) return 0.0f;
    return (x > 0.0f) ? (ax - db) : -(ax - db);
}

// 斜率限制：限制每个控制步的变化量。
float slew_limit(float x, float last, float max_step)
{
    if (max_step <= 0.0f) return x;
    float dx = x - last;
    if (dx >  max_step) return last + max_step;
    if (dx < -max_step) return last - max_step;
    return x;
}
