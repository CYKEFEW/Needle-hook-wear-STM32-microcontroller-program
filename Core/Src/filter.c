#include "filter.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

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

float apply_deadband(float x, float db)
{
    if (db <= 0.0f) return x;
    const float ax = fabsf(x);
    if (ax < db) return 0.0f;
    return (x > 0.0f) ? (ax - db) : -(ax - db);
}

float slew_limit(float x, float last, float max_step)
{
    if (max_step <= 0.0f) return x;
    float dx = x - last;
    if (dx >  max_step) return last + max_step;
    if (dx < -max_step) return last - max_step;
    return x;
}
