#ifndef __filter_H
#define __filter_H

#include "main.h"

typedef struct
{
    float x;    // 状态输出（滤波结果）
    float P;    // 协方差
    float Q;    // 过程噪声
    float R;    // 观测噪声
    float K;    // 卡尔曼增益
} Kalman1D_t;
/*
float a_filter(int16_t Z);
float ab_filter(int16_t Z);
*/
void Kalman1D_Init(Kalman1D_t * const kf, float Q, float R, float init_x);
float kalman_filter(Kalman1D_t * const kf, float Z);

#endif
