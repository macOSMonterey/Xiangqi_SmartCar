#ifndef __Motors_H
#define __Motors_H
#include "stm32f4xx_hal.h"                  // Device header

#define Motor1_PID_KP        680.0f
#define Motor1_PID_KI        0.25f
#define Motor1_PID_KD        0.0f
#define Motor1_PID_MAX_OUT   999.0f
#define Motor1_PID_MAX_IOUT  200.0f

#define Motor2_PID_KP        680.0f
#define Motor2_PID_KI        0.3f
#define Motor2_PID_KD        0.0f
#define Motor2_PID_MAX_OUT   999.0f
#define Motor2_PID_MAX_IOUT  200.0f

#define Motor3_PID_KP        0.0f
#define Motor3_PID_KI        0.0f
#define Motor3_PID_KD        0.0f
#define Motor3_PID_MAX_OUT   2.0f
#define Motor3_PID_MAX_IOUT  0.5f

#define Motor4_PID_KP        0.0f
#define Motor4_PID_KI        0.0f
#define Motor4_PID_KD        0.0f
#define Motor4_PID_MAX_OUT   2.0f
#define Motor4_PID_MAX_IOUT  0.5f

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
    double target_speed;        // 目标速度（设定值）cm/s
    double current_speed;       // 当前速度（测量值）cm/s
    double current_rpm;       // 当前转速（测量值）r/min
    int16_t pwm_output;          // 当前PWM输出值
    
    int16_t encoder_count;  // 当前的编码器读数
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
    pid_type_def  motor_pid;// 包含控制算法
} Motor_t;

// 定义整车运动状态结构体
typedef struct {
    float vx;      // 前后平移速度 (m/s) 前为正
    float vy;      // 左右平移速度 (m/s) 左为正
    float vz;      // 自转速度 (rad/s 或 自定义单位) 逆时针为正
} Chassis_Speed_t;

void Motors_Init(void);

void Encoder_Get(Motor_t *motor);
void Encoder_to_rpm(Motor_t *motor,uint16_t T_ms);
void Encoder_to_Speed(Motor_t *motor,uint16_t T_ms);

void Chassis_Ctrl(void);

#endif
