#include "stm32f4xx_hal.h"                  // Device header
#include "math.h"
#include "Motors.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

Motor_t motor[4];

double rps_1=0;
double rps_2=0;
double rps_3=0;
double rps_4=0;

double v1=0;
double v2=0;
double v3=0;
double v4=0;

struct Kalman_Struct Yaw;

void STBY_1(uint8_t BitValue)
{
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,(GPIO_PinState)BitValue);
}

void STBY_2(uint8_t BitValue)
{
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,(GPIO_PinState)BitValue);
}

void PID_Init(Motor_t *motor, double maxout, double max_iout, double kp, double ki, double kd)
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

	PID_Init(&motor[0],0, 0, 0, 0,0);
	PID_Init(&motor[1],0, 0, 0, 0,0);
	PID_Init(&motor[2],0, 0, 0, 0,0);
	PID_Init(&motor[3],0, 0, 0, 0,0);
	
	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);
}

void PWM_SetCompare(Motor_t* motor)
{
	__HAL_TIM_SET_COMPARE(motor->motor_hw.htim_pwm, motor->motor_hw.pwm_channel,motor->motor_pid.out); 
}

int16_t Encoder_Get(Motor_t *motor)
{
	int16_t Temp;
	Temp = __HAL_TIM_GET_COUNTER(motor->motor_hw.htim_encoder);
	__HAL_TIM_SET_COUNTER(motor->motor_hw.htim_encoder, 0);
	return Temp;
}

void Motor_Ctrl(Motor_t *motor)
{
	if(motor->motor_st.target_speed>0)
	{
		HAL_GPIO_WritePin(motor->motor_hw.dir_port1,motor->motor_hw.dir_pin1,(GPIO_PinState)0);
		HAL_GPIO_WritePin(motor->motor_hw.dir_port2,motor->motor_hw.dir_pin2,(GPIO_PinState)1);
	}
	else
	{
		HAL_GPIO_WritePin(motor->motor_hw.dir_port1,motor->motor_hw.dir_pin1,(GPIO_PinState)1);
		HAL_GPIO_WritePin(motor->motor_hw.dir_port2,motor->motor_hw.dir_pin2,(GPIO_PinState)0);
	}
	PWM_SetCompare(motor);
}

void Read_MotorInfo(void)
{
	for(uint8_t i=0;i<4;i++)
	{
		Encoder_Get(&motor[i]);
	}
}

void Kalman(struct Kalman_Struct *C)
{
	C->P1=(C->P+C->Q);
	C->K=C->P1/(C->P1+C->R);
	C->Angle=(1-0.0025*C->K)*C->PreAngle;
	C->P=(1-C->K)*C->P1;
}

void PID_Update(Motor_t *motor)
{
	motor->motor_pid.fdb=motor->motor_st.current_speed;
	motor->motor_pid.set=motor->motor_st.target_speed;
}

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

double PID_calc(Motor_t *motor, double ref, double set)
{
    motor->motor_pid.aspeed[1] = motor->motor_pid.aspeed[0];
    motor->motor_pid.aspeed[0] = motor->motor_st.current_speed;
    motor->motor_pid.error[2] = motor->motor_pid.error[1];
    motor->motor_pid.error[1] = motor->motor_pid.error[0];
    motor->motor_pid.set = set;
    motor->motor_pid.fdb = ref;
    motor->motor_pid.aspeed[2] = motor->motor_pid.aspeed[1] - motor->motor_pid.aspeed[0];
    motor->motor_pid.error[0] = set - ref;

    motor->motor_pid.Pout = motor->motor_pid.Kp * motor->motor_pid.error[0];
    motor->motor_pid.Iout += motor->motor_pid.Ki * motor->motor_pid.error[0];
    motor->motor_pid.Dbuf[2] = motor->motor_pid.Dbuf[1];
    motor->motor_pid.Dbuf[1] = motor->motor_pid.Dbuf[0];
    motor->motor_pid.Dbuf[0] = (motor->motor_pid.error[0] - motor->motor_pid.error[1]);
    motor->motor_pid.Dout = motor->motor_pid.Kd * motor->motor_pid.Dbuf[0];
    LimitMax(motor->motor_pid.Iout, motor->motor_pid.max_iout);
    motor->motor_pid.out = motor->motor_pid.Pout + motor->motor_pid.Iout + motor->motor_pid.Dout;
    LimitMax(motor->motor_pid.out, motor->motor_pid.max_out);

    return motor->motor_pid.out;
}

void Chassis_Ctrl(void)
{
	for(uint8_t i=0;i<4;i++)
	PID_Update(&motor[i]);
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
		rps_1=(double)Encoder_Get(TIM1)/468.0;
		rps_2=(double)Encoder_Get(TIM2)/468.0;
		rps_3=(double)Encoder_Get(TIM3)/468.0;
		rps_4=(double)Encoder_Get(TIM4)/468.0;
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
