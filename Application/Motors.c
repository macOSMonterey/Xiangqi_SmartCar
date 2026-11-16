#include "stm32f4xx_hal.h"                  // Device header
#include "math.h"
#include "Motors.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

Motor_t motor[4];
Chassis_Speed_t Chassis_Move;

struct Kalman_Struct Yaw;

 /**
  * @brief 限幅函数
  */
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

void STBY_1(uint8_t BitValue)
{
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,(GPIO_PinState)BitValue);
}

void STBY_2(uint8_t BitValue)
{
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,(GPIO_PinState)BitValue);
}

void PID_Init(Motor_t *motor, float maxout, float max_iout, float kp, float ki, float kd)
{
    motor->motor_pid.Kp = kp;
    motor->motor_pid.Ki = ki;
    motor->motor_pid.Kd = kd;
	motor->motor_pid.set = 0;
    motor->motor_pid.max_iout = max_iout;
    motor->motor_pid.max_out = maxout;
}

void Motors_Init(void)
{
	STBY_1(1);
	STBY_2(1);
	motor[0].motor_hw.dir_port1=GPIOC;
	motor[0].motor_hw.dir_pin1=GPIO_PIN_4;
	motor[0].motor_hw.dir_port2=GPIOC;
	motor[0].motor_hw.dir_pin2=GPIO_PIN_5;
	motor[0].motor_hw.htim_encoder=&htim1;
	motor[0].motor_hw.htim_pwm=&htim9;
	motor[0].motor_hw.pwm_channel=TIM_CHANNEL_1;

	motor[1].motor_hw.dir_port1=GPIOB;
	motor[1].motor_hw.dir_pin1=GPIO_PIN_0;
	motor[1].motor_hw.dir_port2=GPIOB;
	motor[1].motor_hw.dir_pin2=GPIO_PIN_1;
	motor[1].motor_hw.htim_encoder=&htim2;
	motor[1].motor_hw.htim_pwm=&htim9;
	motor[1].motor_hw.pwm_channel=TIM_CHANNEL_2;

	motor[2].motor_hw.dir_port1=GPIOE;
	motor[2].motor_hw.dir_pin1=GPIO_PIN_13;
	motor[2].motor_hw.dir_port2=GPIOE;
	motor[2].motor_hw.dir_pin2=GPIO_PIN_14;
	motor[2].motor_hw.htim_encoder=&htim3;
	motor[2].motor_hw.htim_pwm=&htim12;
	motor[2].motor_hw.pwm_channel=TIM_CHANNEL_1;

	motor[3].motor_hw.dir_port1=GPIOE;
	motor[3].motor_hw.dir_pin1=GPIO_PIN_15;
	motor[3].motor_hw.dir_port2=GPIOB;
	motor[3].motor_hw.dir_pin2=GPIO_PIN_10;
	motor[3].motor_hw.htim_encoder=&htim4;
	motor[3].motor_hw.htim_pwm=&htim12;
	motor[3].motor_hw.pwm_channel=TIM_CHANNEL_2;

	PID_Init(&motor[0],Motor1_PID_MAX_OUT, Motor1_PID_MAX_IOUT, Motor1_PID_KP, Motor1_PID_KI,Motor1_PID_KD);
	PID_Init(&motor[1],Motor2_PID_MAX_OUT, Motor2_PID_MAX_IOUT, Motor2_PID_KP, Motor2_PID_KI,Motor2_PID_KD);
	PID_Init(&motor[2],Motor3_PID_MAX_OUT, Motor3_PID_MAX_IOUT, Motor3_PID_KP, Motor3_PID_KI,Motor3_PID_KD);
	PID_Init(&motor[3],Motor4_PID_MAX_OUT, Motor4_PID_MAX_IOUT, Motor4_PID_KP, Motor4_PID_KI,Motor4_PID_KD);
	
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL); 
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); 
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); 
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); 

	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);
}

void PWM_SetCompare(Motor_t* motor)
{
	__HAL_TIM_SET_COMPARE(motor->motor_hw.htim_pwm, motor->motor_hw.pwm_channel,motor->motor_st.pwm_output); 
}

void Encoder_Get(Motor_t *motor)
{
	motor->motor_st.encoder_count = (int16_t)__HAL_TIM_GET_COUNTER(motor->motor_hw.htim_encoder);
	__HAL_TIM_SET_COUNTER(motor->motor_hw.htim_encoder, 0);
}

void Encoder_to_rpm(Motor_t *motor,uint16_t T_ms)
{
	if (T_ms == 0) return;
	motor->motor_st.current_rpm=motor->motor_st.encoder_count/4680.0/(T_ms/60000.0);
}

void Encoder_to_Speed(Motor_t *motor,uint16_t T_ms)
{
	motor->motor_st.current_speed=motor->motor_st.encoder_count/4680.0/(T_ms/1000.0)*M_PI*0.06*100;
}

void Motor_Ctrl(Motor_t *motor)
{
	if(motor->motor_pid.out>0)
	{
		HAL_GPIO_WritePin(motor->motor_hw.dir_port1,motor->motor_hw.dir_pin1,GPIO_PIN_SET);
		HAL_GPIO_WritePin(motor->motor_hw.dir_port2,motor->motor_hw.dir_pin2,GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(motor->motor_hw.dir_port1,motor->motor_hw.dir_pin1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor->motor_hw.dir_port2,motor->motor_hw.dir_pin2,GPIO_PIN_SET);
	}
	motor->motor_st.pwm_output=(uint16_t)round(fabs(motor->motor_pid.out));
	PWM_SetCompare(motor);
}

void Chassis_Kinematics(Chassis_Speed_t *speed, Motor_t *motor)
{
    // 麦轮旋转系数，需要根据你的车轮大小和底盘尺寸实际测量调整
    // 假设你的轮子半径 R=0.03m，L+W 约等于 0.2m，这个系数要自己试出来，先给个 0.15 左右
    float K_rotate = 0.15f;
    // 麦轮解算
    // 左前轮
    motor[0].motor_st.target_speed = 5;//speed->vx - speed->vy - speed->vz * K_rotate;
    // 左后轮
    motor[1].motor_st.target_speed = 5;//speed->vx + speed->vy - speed->vz * K_rotate;
    // 右后轮
    motor[2].motor_st.target_speed = speed->vx - speed->vy + speed->vz * K_rotate;
    // 右前轮
    motor[3].motor_st.target_speed = speed->vx + speed->vy + speed->vz * K_rotate;

	LimitMax(motor[0].motor_st.target_speed, 50);
	LimitMax(motor[1].motor_st.target_speed, 50);
	LimitMax(motor[2].motor_st.target_speed, 50);
	LimitMax(motor[3].motor_st.target_speed, 50);
}

void Kalman(struct Kalman_Struct *C)
{
	C->P1=(C->P+C->Q);
	C->K=C->P1/(C->P1+C->R);
	C->Angle=(1-0.0025*C->K)*C->PreAngle;
	C->P=(1-C->K)*C->P1;
}

void PID_calc(Motor_t *motor)
{
    motor->motor_pid.aspeed[1] = motor->motor_pid.aspeed[0];
    motor->motor_pid.aspeed[0] = motor->motor_st.current_speed;
    motor->motor_pid.error[2] = motor->motor_pid.error[1];
    motor->motor_pid.error[1] = motor->motor_pid.error[0];
	motor->motor_pid.fdb=motor->motor_st.current_speed;
	motor->motor_pid.set=motor->motor_st.target_speed;
    motor->motor_pid.aspeed[2] = motor->motor_pid.aspeed[1] - motor->motor_pid.aspeed[0];
    motor->motor_pid.error[0] = motor->motor_pid.set-motor->motor_pid.fdb;

    motor->motor_pid.Pout = motor->motor_pid.Kp * motor->motor_pid.error[0];
    motor->motor_pid.Iout += motor->motor_pid.Ki * motor->motor_pid.error[0];
    motor->motor_pid.Dbuf[2] = motor->motor_pid.Dbuf[1];
    motor->motor_pid.Dbuf[1] = motor->motor_pid.Dbuf[0];
    motor->motor_pid.Dbuf[0] = (motor->motor_pid.error[0] - motor->motor_pid.error[1]);
    motor->motor_pid.Dout = motor->motor_pid.Kd * motor->motor_pid.Dbuf[0];
    LimitMax(motor->motor_pid.Iout, motor->motor_pid.max_iout);
    motor->motor_pid.out = motor->motor_pid.Pout + motor->motor_pid.Iout + motor->motor_pid.Dout;
    LimitMax(motor->motor_pid.out, motor->motor_pid.max_out);
}

void Chassis_Ctrl(void)
{
	Chassis_Kinematics(&Chassis_Move,motor);
	for(uint8_t i=0;i<4;i++)
	PID_calc(&motor[i]);
	for(uint8_t i=0;i<4;i++)
	Motor_Ctrl(&motor[i]);
}

/*
void Encoder_Task(void * pvParameters)
{
	PID_Init(&PID_1,40,0.2,0.1);
	PID_Init(&PID_2,40,0.2,0.1);
	PID_Init(&PID_3,40,0.2,0.1);
	PID_Init(&PID_4,40,0.2,0.1);
	TickType_t xLastWakeTime = xTaskGetTickCount();
	TickType_t xFrequency =pdMS_TO_TICKS(100);
	while(1)
	{
		rps_1=(float)Encoder_Get(TIM1)/468.0;
		rps_2=(float)Encoder_Get(TIM2)/468.0;
		rps_3=(float)Encoder_Get(TIM3)/468.0;
		rps_4=(float)Encoder_Get(TIM4)/468.0;
		Motor_1(PID_Value(&PID_1,v1/3.14159/0.06,rps_1));
		Motor_2(PID_Value(&PID_2,v2/3.14159/0.06,rps_2));
		Motor_3(PID_Value(&PID_3,v3/3.14159/0.06,rps_3));
		Motor_4(PID_Value(&PID_4,v4/3.14159/0.06,rps_4));
		vTaskDelayUntil(&xLastWakeTime,xFrequency);
	}
}/*
/*
void Motor_Task(void * pvParameters)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	TickType_t xFrequency =pdMS_TO_TICKS(200);
	while(1)
	{
		if(Ultrasonic_Mode)
		{
			if(U_INFO.Angle<Target_Dist)
			{
				Flag=1;
				Car.X=0;
				Car.Y=-1.2;
				Car.Z=0;
			}
			else if(Flag==1)
			{
				Flag=0;
				Car.X=0;
				Car.Y=0;
				Car.Z=0;
			}
		}
		v1=Car.X+Car.Y+0.17*Car.Z/0.03;
		v2=-Car.X+Car.Y-0.17*Car.Z/0.03;
		v3=-Car.X+Car.Y+0.17*Car.Z/0.03;
		v4=Car.X+Car.Y-0.17*Car.Z/0.03;
		vTaskDelayUntil(&xLastWakeTime,xFrequency);
	}
}
	*/
