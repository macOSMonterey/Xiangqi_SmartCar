#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"                  // Device header
#include "MPU6050_Reg.h"
#include "MPU6050.h"
#include "i2c.h"
#include <stdint.h>

#define MPU6050_ADDRESS 0xD0
#define g 9.80665f

MPU6050_t MPU6050;

void MPU6050_WriteReg(uint8_t RegAddress,uint8_t Data)
{
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, RegAddress, I2C_MEMADD_SIZE_8BIT, &Data, 1, 100);
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS, RegAddress, I2C_MEMADD_SIZE_8BIT, &Data, 1, 100);
	return Data;
}

void MPU6050_Init(void)
{
	MPU6050.config.Sample_Rate=100;
	MPU6050.config.dlpf_cfg=0x04;
	MPU6050.config.accel_afs_sel=0x08;
	MPU6050.config.gyro_fs_sel=0x08;
	MPU6050.config.accel_scale=8192.0f;
	MPU6050.config.gyro_scale=65.536f;
	MPU6050.config.temp_scale=340.0f;
	MPU6050.config.temp_offset=36.53f;
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1,0x01);
	osDelay(50);
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2,0x00);
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV,(uint8_t)(1000 / MPU6050.config.Sample_Rate - 1));
	MPU6050_WriteReg(MPU6050_CONFIG,MPU6050.config.dlpf_cfg);
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG,MPU6050.config.gyro_fs_sel);
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG,MPU6050.config.accel_afs_sel);
}

void MPU6050_GetID(MPU6050_t *MPU6050)
{
	MPU6050->rawdata.ID=MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

void MPU6050_ReadAccX(MPU6050_t *MPU6050)
{
	uint16_t Data_H,Data_L;
	Data_H=MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
	Data_L=MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
	MPU6050->rawdata.Accel_X=(int16_t)((Data_H<<8)|Data_L);
	MPU6050->data.Accel_X=(int16_t)((Data_H<<8)|Data_L)/MPU6050->config.accel_scale*g;
}

void MPU6050_ReadAccY(MPU6050_t *MPU6050)
{
	uint16_t Data_H,Data_L;
	Data_H=MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
	Data_L=MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
	MPU6050->rawdata.Accel_Y=(int16_t)((Data_H<<8)|Data_L);
	MPU6050->data.Accel_Y=(int16_t)((Data_H<<8)|Data_L)/MPU6050->config.accel_scale*g;
}

void MPU6050_ReadAccZ(MPU6050_t *MPU6050)
{
	uint16_t Data_H,Data_L;
	Data_H=MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
	Data_L=MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
	MPU6050->rawdata.Accel_Z=(int16_t)((Data_H<<8)|Data_L);
	MPU6050->data.Accel_Z=(int16_t)((Data_H<<8)|Data_L)/MPU6050->config.accel_scale*g;
}

void MPU6050_ReadGyroX(MPU6050_t *MPU6050)
{
	uint16_t Data_H,Data_L;
	Data_H=MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
	Data_L=MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
	MPU6050->rawdata.Gyro_X=(int16_t)((Data_H<<8)|Data_L);
	MPU6050->data.Gyro_X=(int16_t)((Data_H<<8)|Data_L)/MPU6050->config.gyro_scale;
}

void MPU6050_ReadGyroY(MPU6050_t *MPU6050)
{
	uint16_t Data_H,Data_L;
	Data_H=MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
	Data_L=MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
	MPU6050->rawdata.Gyro_Y=(int16_t)((Data_H<<8)|Data_L);
	MPU6050->data.Gyro_Y=(int16_t)((Data_H<<8)|Data_L)/MPU6050->config.gyro_scale;
}

void MPU6050_ReadGyroZ(MPU6050_t *MPU6050)
{
	uint16_t Data_H,Data_L;
	Data_H=MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
	Data_L=MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
	MPU6050->rawdata.Gyro_Z=(int16_t)((Data_H<<8)|Data_L);
	MPU6050->data.Gyro_Z=(int16_t)((Data_H<<8)|Data_L)/MPU6050->config.gyro_scale;
}

void MPU6050_ReadTemp(MPU6050_t *MPU6050)
{
	uint16_t Data_H,Data_L;
	Data_H=MPU6050_ReadReg(MPU6050_TEMP_OUT_H);
	Data_L=MPU6050_ReadReg(MPU6050_TEMP_OUT_L);
	MPU6050->rawdata.Temp=(int16_t)((Data_H<<8)|Data_L);
	MPU6050->data.Temp=(int16_t)((Data_H<<8)|Data_L)/MPU6050->config.temp_scale+MPU6050->config.temp_offset;
}

/* 校准函数：调用此函数时，务必保证板子【绝对静止、水平平放】 */
void MPU6050_Calibrate(MPU6050_t *MPU6050,uint16_t samples)
{
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;

    for (int i = 0; i < samples; i++)
    {
		MPU6050_ReadAccX(MPU6050);
		MPU6050_ReadAccY(MPU6050);
		MPU6050_ReadAccZ(MPU6050);
		MPU6050_ReadGyroX(MPU6050);
		MPU6050_ReadGyroY(MPU6050);
		MPU6050_ReadGyroZ(MPU6050);
		MPU6050_ReadTemp(MPU6050);

        ax_sum += MPU6050->rawdata.Accel_X;
        ay_sum += MPU6050->rawdata.Accel_Y;
        // 加速度 Z 轴比较特殊：理想水平放置时，它应该输出 1g（即 8192）
        // 所以我们要算的误差是：当前读数与 8192 的差值
        az_sum += (MPU6050->rawdata.Accel_Z - 8192); 
        
        gx_sum += MPU6050->rawdata.Gyro_X;
        gy_sum += MPU6050->rawdata.Gyro_Y;
        gz_sum += MPU6050->rawdata.Gyro_Z;
        
        osDelay(2); // 采样间隔 2ms
    }

    // 1. 计算加速度零偏 (LSB)
    MPU6050->config.accel_offset_x = ax_sum / samples;
    MPU6050->config.accel_offset_y = ay_sum / samples;
    MPU6050->config.accel_offset_z = az_sum / samples;

    // 2. 计算陀螺仪零偏 (直接存为物理量 °/s，方便后续直接减)
    // 注意：这里除以 gyro_scale 就变成 °/s 了
    MPU6050->config.gyro_offset_x = gx_sum / samples;
    MPU6050->config.gyro_offset_y = gy_sum / samples;
    MPU6050->config.gyro_offset_z = gz_sum / samples;
}
