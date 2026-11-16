#ifndef __MPU6050_H
#define __MPU6050_H
#include "stm32f4xx_hal.h"                  // Device header

#define MPU6050_accel_offset_x    0.0f
#define MPU6050_accel_offset_y    0.0f
#define MPU6050_accel_offset_z    0.0f
#define MPU6050_gyro_offset_x     0.0f
#define MPU6050_gyro_offset_y     0.0f
#define MPU6050_gyro_offset_z     0.0f

/* 传感器数据结构体 */
typedef struct {
    uint8_t ID;
    int16_t Accel_X;
    int16_t Accel_Y;
    int16_t Accel_Z;
    int16_t Temp;
    int16_t Gyro_X;
    int16_t Gyro_Y;
    int16_t Gyro_Z;
} MPU6050_RawData_t;

typedef struct {
    float Accel_X;
    float Accel_Y;
    float Accel_Z;
    float Temp;
    float Gyro_X;
    float Gyro_Y;
    float Gyro_Z;
} MPU6050_Data_t;

typedef struct {
    float Accel_X;
    float Accel_Y;
    float Accel_Z;
    float Temp;
    float Gyro_X;
    float Gyro_Y;
    float Gyro_Z;
} MPU6050_filteredData_t;

typedef struct {
    uint8_t Sample_Rate;     // 采样率 (Hz)
    uint8_t dlpf_cfg;        // 低通滤波 (0x1A)

    uint8_t gyro_fs_sel;     // 陀螺仪量程 (0x1B)
    float gyro_offset_x;   // 校准得出的偏置
    float gyro_offset_y;
    float gyro_offset_z;
    float gyro_scale;        // 对应的陀螺仪换算系数

    uint8_t accel_afs_sel;   // 加速度量程 (0x1C)
    float accel_offset_x;  // 校准得出的偏置
    float accel_offset_y;
    float accel_offset_z;
    float accel_scale;       // 对应的加速度换算系数

    float temp_scale;        // 对应的温度换算系数
    float temp_offset;       // 对应的温度偏移系数
} MPU6050_Config_t;

/* 设备管理结构体 */
typedef struct {
    MPU6050_RawData_t rawdata;     // 原始的数据结构
    MPU6050_Data_t data;           // 解析的数据结构
    MPU6050_filteredData_t filteredData;//滤波后的数据结构
    MPU6050_Config_t config;       // 配置结构
} MPU6050_t;

void MPU6050_Init(void);
void MPU6050_ConfigInit(void);

HAL_StatusTypeDef MPU6050_Calibrate(MPU6050_t *MPU6050,uint16_t samples);
HAL_StatusTypeDef MPU6050_Recover(MPU6050_t *mpu);

HAL_StatusTypeDef MPU6050_GetID(MPU6050_t *MPU6050);
HAL_StatusTypeDef MPU6050_ReadAccX(MPU6050_t *MPU6050);
HAL_StatusTypeDef MPU6050_ReadAccY(MPU6050_t *MPU6050);
HAL_StatusTypeDef MPU6050_ReadAccZ(MPU6050_t *MPU6050);
HAL_StatusTypeDef MPU6050_ReadGyroX(MPU6050_t *MPU6050);
HAL_StatusTypeDef MPU6050_ReadGyroY(MPU6050_t *MPU6050);
HAL_StatusTypeDef MPU6050_ReadGyroZ(MPU6050_t *MPU6050);
HAL_StatusTypeDef MPU6050_ReadTemp(MPU6050_t *MPU6050);

#endif
