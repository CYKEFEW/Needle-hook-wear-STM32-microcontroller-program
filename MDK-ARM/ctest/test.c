#include "stdint.h"
#include "stdio.h"

#define     __IO    volatile

int main() {
    float a = 1.5;
    float *ptr = &a;
    uint32_t b = *(uint32_t*)ptr;

    printf("%f\n", a);
    printf("%f\n", *ptr);
    printf("%x\n", b);

    *ptr = 2.5;
    printf("%f\n", a);

    uint32_t *ptr2 = &b;
    a = *(float*)ptr2;
    printf("%f\n", a);
    return 0;
}