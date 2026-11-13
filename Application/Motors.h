#ifndef __Motors_H
#define __Motors_H
#include "stm32f4xx_hal.h"                  // Device header


#define Mem_LVGL 1024
#define Mem_KEY 512
#define Mem_fps 256
#define Mem_Encoder 256
#define Mem_MPU6050 256
#define Mem_Motor 512
#define Mem_Ultrasonic 256
#define Mem_CPUUsage 512
#define Mem_BlueTooth 256
#define Mem_BT_Check 512

struct Kalman_Struct
{
	double Angle;
    double PreAngle;
    double P1;
	double P;
    double Q;
    double K;
	double R;
};

// ================= 第一层：硬件接口层 =================
// 负责管理所有的物理引脚、定时器句柄
typedef struct {
    TIM_HandleTypeDef *htim_pwm;     // PWM定时器句柄
    uint32_t pwm_channel;            // PWM通道
    
    GPIO_TypeDef *dir_port1;         // 方向引脚1端口
    uint16_t dir_pin1;               // 方向引脚1
    GPIO_TypeDef *dir_port2;         // 方向引脚2端口
    uint16_t dir_pin2;               // 方向引脚2
    
    TIM_HandleTypeDef *htim_encoder; // 编码器定时器句柄
} Motor_Hardware_t;

// ================= 第二层：状态数据层 =================
// 负责记录电机的目标值、实际值、累计里程等
typedef struct {
    int16_t target_speed;        // 目标速度（设定值）
    int16_t current_speed;       // 当前速度（测量值）
    int16_t pwm_output;          // 当前PWM输出值
    
    int32_t last_encoder_count;  // 上一次的编码器读数
    int32_t total_encoder_count; // 累计编码器脉冲数（可算里程）
} Motor_Status_t;

// ================= 第三层：闭环控制层 =================
// 负责PID算法相关的参数和状态
typedef struct
{
    uint8_t mode;
    //PID 三参数
    double Kp;
    double Ki;
    double Kd;

    double max_out;  //最大输出
    double max_iout; //最大积分输出

    double set;
    double fdb;
	double aspeed[3];

    double out;
    double Pout;
    double Iout;
    double Dout;
    double Dbuf[3];  //微分项 0最新 1上一次 2上上次
    double error[3]; //误差项 0最新 1上一次 2上上次

} pid_type_def;

// ================= 主电机结构体（嵌套以上三层）=================
typedef struct {
    Motor_Hardware_t motor_hw;  // 包含硬件接口
    Motor_Status_t   motor_st;  // 包含状态数据
    pid_type_def  motor_pid// 包含控制算法
} Motor_t;

void Motors_Init(void);
void Motor_1(int16_t Speed);
void Motor_2(int16_t Speed);
void Motor_3(int16_t Speed);
void Motor_4(int16_t Speed);

void Encoder_Task(void * pvParameters);
void MPU6050_Task(void * pvParameters);
void Ultrasonic_Task(void * pvParameters);
void Motor_Task(void * pvParameters);

#endif
