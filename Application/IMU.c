#include "IMU.h"
#include "math.h"
#include "string.h"

static float wrap_pi(float ang)
{
    while(ang >  M_PI) ang -= 2.0f * M_PI;
    while(ang < -M_PI) ang += 2.0f * M_PI;
    return ang;
}

static void mat_eye2(float m[2][2])
{
    m[0][0]=1;m[0][1]=0;
    m[1][0]=0;m[1][1]=1;
}

static void mat_mul22(const float A[2][2], const float B[2][2], float C[2][2])
{
    float t[2][2]={0};
    t[0][0]=A[0][0]*B[0][0]+A[0][1]*B[1][0];
    t[0][1]=A[0][0]*B[0][1]+A[0][1]*B[1][1];
    t[1][0]=A[1][0]*B[0][0]+A[1][1]*B[1][0];
    t[1][1]=A[1][0]*B[0][1]+A[1][1]*B[1][1];
    memcpy(C,t,sizeof(t));
}

static void mat_mul21(const float A[2][2], const float B[2], float C[2])
{
    float t[2]={0};
    t[0]=A[0][0]*B[0]+A[0][1]*B[1];
    t[1]=A[1][0]*B[0]+A[1][1]*B[1];
    memcpy(C,t,sizeof(t));
}

static void mat_mul12(const float A[2], const float B[2][2], float C[2])
{
    float t[2]={0};
    t[0]=A[0]*B[0][0]+A[1]*B[1][0];
    t[1]=A[0]*B[0][1]+A[1]*B[1][1];
    memcpy(C,t,sizeof(t));
}

static void mat_add22(const float A[2][2], const float B[2][2], float C[2][2])
{
    for(int i=0;i<2;i++)
        for(int j=0;j<2;j++)
            C[i][j]=A[i][j]+B[i][j];
}

void YawKF2_Init(YawKF2_t *kf, float dt, float Q_yaw, float Q_bias, float R_yaw)
{
    memset(kf,0,sizeof(YawKF2_t));
    kf->dt = dt;
    kf->R = R_yaw;

    kf->Q[0][0] = Q_yaw;
    kf->Q[1][1] = Q_bias;

    mat_eye2(kf->P);
    kf->P[0][0] = 0.1f;
    kf->P[1][1] = 0.001f;
}

static void YawKF2_Predict(YawKF2_t *kf, float wz)
{
    float dt = kf->dt;
    float *X = kf->X;
    float yaw = X[0];
    float bias = X[1];

    // 状态预测
    X[0] = wrap_pi(yaw + (wz - bias)*dt);
    X[1] = bias;

    // F
    float F[2][2] = {
        {1, -dt},
        {0, 1}
    };
    float Ft[2][2] = {
        {1, 0},
        {-dt,1}
    };
    float FP[2][2], FPFt[2][2];
    mat_mul22(F, kf->P, FP);
    mat_mul22(FP, Ft, FPFt);
    mat_add22(FPFt, kf->Q, kf->P);
}

static void YawKF2_Update(YawKF2_t *kf, float yaw_obs)
{
    float *X = kf->X;
    float err = wrap_pi(yaw_obs - X[0]);

    float H[2] = {1,0};
    float HP[2];
    HP[0] = H[0]*kf->P[0][0] + H[1]*kf->P[1][0];
    HP[1] = H[0]*kf->P[0][1] + H[1]*kf->P[1][1];

    float S = HP[0] * H[0] + HP[1] * H[1] + kf->R;
    float invS = 1.0f / S;

    float K[2];
    K[0] = HP[0] * invS;
    K[1] = HP[1] * invS;

    // 修正状态
    X[0] = wrap_pi(X[0] + K[0] * err);
    X[1] = X[1] + K[1] * err;

    // P = (I-KH)P
    float KH[2][2];
    KH[0][0]=K[0]*H[0]; KH[0][1]=K[0]*H[1];
    KH[1][0]=K[1]*H[0]; KH[1][1]=K[1]*H[1];
    float I_KH[2][2];
    mat_eye2(I_KH);
    I_KH[0][0]-=KH[0][0]; I_KH[0][1]-=KH[0][1];
    I_KH[1][0]-=KH[1][0]; I_KH[1][1]-=KH[1][1];

    float tmp[2][2];
    mat_mul22(I_KH, kf->P, tmp);
    memcpy(kf->P, tmp, sizeof(tmp));
}

/**
 * @param z[0] wz rad/s
 * @param z[1] yaw_obs rad
 * @param obs_valid 1有观测，0只预测
 */
void YawKF2_Run(YawKF2_t *kf, float z[2], uint8_t obs_valid)
{
    YawKF2_Predict(kf, z[0]);
    if(obs_valid)
    {
        YawKF2_Update(kf, z[1]);
    }
}
