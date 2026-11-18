#ifndef __IMU_H
#define __IMU_H

#include "stm32f4xx_hal.h"

typedef struct
{
    float X[2];   // X[0]=yaw, X[1]=gyro_bias
    float P[2][2];
    float Q[2][2];
    float R;
    float dt;
} YawKF2_t;

void YawKF2_Init(YawKF2_t *kf, float dt, float Q_yaw, float Q_bias, float R_yaw);
void YawKF2_Run(YawKF2_t *kf, float z[2], uint8_t obs_valid);

#endif
