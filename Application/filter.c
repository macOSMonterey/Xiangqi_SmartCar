#include "filter.h"
/**
float a_filter(int16_t Z)
{
	static float x=0;
	x=x+0.5*(Z-x);
//	HAL_Delay(1);
	return x;
}

float ab_filter(int16_t Z)
{
	static float x=0,v=0;
	x=x+0.4*(Z-x);
	v=v+1.5*(Z-x);
	x=x+v*0.001;
//	HAL_Delay(1);
	return x;
}
*/
Kalman1D_t Kalman1D_AX,Kalman1D_AY,Kalman1D_AZ,Kalman1D_GX,Kalman1D_GY,Kalman1D_GZ;

void Kalman1D_Init(Kalman1D_t * const kf, float Q, float R, float init_x)
{
    if(kf == NULL) return;  // 空指针保护
    kf->x = init_x;
    kf->P = 1.0f;
    kf->Q = Q;
    kf->R = R;
    kf->K = 0.0f;
}

float kalman_filter(Kalman1D_t * const kf, float Z)
{
    if(kf == NULL) return 0.0f;

    // 预测协方差
    kf->P = kf->P + kf->Q;
    // 卡尔曼增益
    kf->K = kf->P / (kf->P + kf->R);
    // 状态更新
    kf->x = kf->x + kf->K * (Z - kf->x);
    // 协方差更新
    kf->P = (1.0f - kf->K) * kf->P;

    return kf->x;
}
