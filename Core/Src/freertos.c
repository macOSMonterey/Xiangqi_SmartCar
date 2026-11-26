/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU6050.h"
#include "Motors.h"
#include <stdint.h>
#include "i2c.h"
#include "filter.h"
#include "IMU.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KEY_Pin       GPIO_PIN_4
#define KEY_GPIO_Port GPIOE
#define DEG2RAD (M_PI / 180.0f)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern Motor_t motor[4];
extern MPU6050_t MPU6050;
extern Kalman1D_t Kalman1D_AX,Kalman1D_AY,Kalman1D_AZ,Kalman1D_GX,Kalman1D_GY,Kalman1D_GZ;
YawKF2_t yaw_kf;
static volatile uint32_t idle_tick = 0;
TickType_t ulLastIdleTick = 0;
float cpu_usage;
/* USER CODE END Variables */
/* Definitions for ReadMotor_Task */
osThreadId_t ReadMotor_TaskHandle;
const osThreadAttr_t ReadMotor_Task_attributes = {
  .name = "ReadMotor_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Chassis_Task */
osThreadId_t Chassis_TaskHandle;
const osThreadAttr_t Chassis_Task_attributes = {
  .name = "Chassis_Task",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SerialParse_Task */
osThreadId_t SerialParse_TaskHandle;
const osThreadAttr_t SerialParse_Task_attributes = {
  .name = "SerialParse_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for IMU_Task */
osThreadId_t IMU_TaskHandle;
const osThreadAttr_t IMU_Task_attributes = {
  .name = "IMU_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MPUmeasure_Task */
osThreadId_t MPUmeasure_TaskHandle;
const osThreadAttr_t MPUmeasure_Task_attributes = {
  .name = "MPUmeasure_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Key_Task */
osThreadId_t Key_TaskHandle;
const osThreadAttr_t Key_Task_attributes = {
  .name = "Key_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for referee_task */
osThreadId_t referee_taskHandle;
const osThreadAttr_t referee_task_attributes = {
  .name = "referee_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for cpu_monitor_task */
osThreadId_t cpu_monitor_taskHandle;
const osThreadAttr_t cpu_monitor_task_attributes = {
  .name = "cpu_monitor_task",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SerialData */
osMessageQueueId_t SerialDataHandle;
const osMessageQueueAttr_t SerialData_attributes = {
  .name = "SerialData"
};
/* Definitions for MPU6050_Mutex */
osMutexId_t MPU6050_MutexHandle;
const osMutexAttr_t MPU6050_Mutex_attributes = {
  .name = "MPU6050_Mutex"
};
/* Definitions for sem_KeyPressed */
osSemaphoreId_t sem_KeyPressedHandle;
const osSemaphoreAttr_t sem_KeyPressed_attributes = {
  .name = "sem_KeyPressed"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void ReadMotor(void *argument);
void Chassis(void *argument);
void SerialParse(void *argument);
void IMU(void *argument);
void MPUmeasure(void *argument);
void Key(void *argument);
extern void referee_usart_task(void *argument);
void cpu_monitor(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook(void) 
{
  TickType_t ulCurrentTick = xTaskGetTickCount();
  if(ulCurrentTick > ulLastIdleTick)
  {
      idle_tick++;
      ulLastIdleTick = ulCurrentTick;
  }
  /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
  to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
  task. It is essential that code added to this hook function never attempts
  to block in any way (for example, call xQueueReceive() with a block time
  specified, or call vTaskDelay()). If the application makes use of the
  vTaskDelete() API function (as this demo application does) then it is also
  important that vApplicationIdleHook() is permitted to return to its calling
  function, because it is the responsibility of the idle task to clean up
  memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of MPU6050_Mutex */
  MPU6050_MutexHandle = osMutexNew(&MPU6050_Mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of sem_KeyPressed */
  sem_KeyPressedHandle = osSemaphoreNew(1, 0, &sem_KeyPressed_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of SerialData */
  SerialDataHandle = osMessageQueueNew (1024, sizeof(uint8_t), &SerialData_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ReadMotor_Task */
  ReadMotor_TaskHandle = osThreadNew(ReadMotor, NULL, &ReadMotor_Task_attributes);

  /* creation of Chassis_Task */
  Chassis_TaskHandle = osThreadNew(Chassis, NULL, &Chassis_Task_attributes);

  /* creation of SerialParse_Task */
  SerialParse_TaskHandle = osThreadNew(SerialParse, NULL, &SerialParse_Task_attributes);

  /* creation of IMU_Task */
  IMU_TaskHandle = osThreadNew(IMU, NULL, &IMU_Task_attributes);

  /* creation of MPUmeasure_Task */
  MPUmeasure_TaskHandle = osThreadNew(MPUmeasure, NULL, &MPUmeasure_Task_attributes);

  /* creation of Key_Task */
  Key_TaskHandle = osThreadNew(Key, NULL, &Key_Task_attributes);

  /* creation of referee_task */
  referee_taskHandle = osThreadNew(referee_usart_task, NULL, &referee_task_attributes);

  /* creation of cpu_monitor_task */
  cpu_monitor_taskHandle = osThreadNew(cpu_monitor, NULL, &cpu_monitor_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_ReadMotor */
/**
 * @brief  Function implementing the ReadMotor_Task thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_ReadMotor */
void ReadMotor(void *argument)
{
  /* USER CODE BEGIN ReadMotor */
  Motors_Init();
  uint8_t i = 0;
  uint32_t tick = osKernelGetTickCount();
  uint8_t T_ms = 20;
  /* Infinite loop */
  for (;;) {
    for (i = 0; i < 4; i++) {
      Encoder_Get(&motor[i]);
      Encoder_to_rpm(&motor[i], T_ms);
      Encoder_to_Speed(&motor[i], T_ms);
    }
    tick += pdMS_TO_TICKS(T_ms);
    osDelayUntil(tick);
  }
  /* USER CODE END ReadMotor */
}

/* USER CODE BEGIN Header_Chassis */
/**
 * @brief Function implementing the ChassisTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Chassis */
void Chassis(void *argument)
{
  /* USER CODE BEGIN Chassis */
  /* Infinite loop */
  for (;;) {
    // Chassis_Ctrl();
    osDelay(20);
  }
  /* USER CODE END Chassis */
}

/* USER CODE BEGIN Header_SerialParse */
/**
 * @brief Function implementing the SerialParseTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_SerialParse */
void SerialParse(void *argument)
{
  /* USER CODE BEGIN SerialParse */
  /* Infinite loop */
  for (;;) {
    osDelay(10);
  }
  /* USER CODE END SerialParse */
}

/* USER CODE BEGIN Header_IMU */
/**
 * @brief Function implementing the IMU_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_IMU */
void IMU(void *argument)
{
  /* USER CODE BEGIN IMU */
  float z[2];
  YawKF2_Init(&yaw_kf, 0.01f, 1e-4f, 1e-7f, 0.01f);
  /* Infinite loop */
  for (;;) {
    z[0]=MPU6050.filteredData.Gyro_Z*DEG2RAD;
    z[1]=yaw_kf.X[0];
    YawKF2_Run(&yaw_kf, z, 1);
    osDelay(10);
  }
  /* USER CODE END IMU */
}

/* USER CODE BEGIN Header_MPUmeasure */
/**
 * @brief Function implementing the MPUmeasure_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_MPUmeasure */
void MPUmeasure(void *argument)
{
  /* USER CODE BEGIN MPUmeasure */
  Kalman1D_Init(&Kalman1D_AX, 0.2, 2.0, 0);
  Kalman1D_Init(&Kalman1D_AY, 0.2, 2.0, 0);
  Kalman1D_Init(&Kalman1D_AZ, 0.2, 2.0, 0);
  Kalman1D_Init(&Kalman1D_GX, 0.2, 2.0, 0);
  Kalman1D_Init(&Kalman1D_GY, 0.2, 2.0, 0);
  Kalman1D_Init(&Kalman1D_GZ, 0.2, 2.0, 0);
  MPU6050_ConfigInit();
  MPU6050_Init();
  MPU6050_GetID(&MPU6050);
  HAL_StatusTypeDef ret = HAL_OK;
  uint32_t tick = osKernelGetTickCount();
  uint32_t T_ms = 1000 / MPU6050.config.Sample_Rate;
  uint8_t fail_count = 0; // 连续失败计数器
  /* Infinite loop */
  for (;;) {
    osMutexAcquire(MPU6050_MutexHandle, osWaitForever);
    tick = osKernelGetTickCount();
    ret = MPU6050_ReadAccX(&MPU6050);
    if (ret == HAL_OK) ret = MPU6050_ReadAccY(&MPU6050);
    if (ret == HAL_OK) ret = MPU6050_ReadAccZ(&MPU6050);
    if (ret == HAL_OK) ret = MPU6050_ReadGyroX(&MPU6050);
    if (ret == HAL_OK) ret = MPU6050_ReadGyroY(&MPU6050);
    if (ret == HAL_OK) ret = MPU6050_ReadGyroZ(&MPU6050);
    if (ret == HAL_OK) ret = MPU6050_ReadTemp(&MPU6050);
    osMutexRelease(MPU6050_MutexHandle);
    if (ret != HAL_OK) 
    {
      fail_count++;
    }
    else
    {
      fail_count=0;
      MPU6050.filteredData.Accel_X=kalman_filter(&Kalman1D_AX, MPU6050.data.Accel_X);
      MPU6050.filteredData.Accel_Y=kalman_filter(&Kalman1D_AY, MPU6050.data.Accel_Y);
      MPU6050.filteredData.Accel_Z=kalman_filter(&Kalman1D_AZ, MPU6050.data.Accel_Y);
      MPU6050.filteredData.Gyro_X=kalman_filter(&Kalman1D_GX, MPU6050.data.Gyro_X);
      MPU6050.filteredData.Gyro_Y=kalman_filter(&Kalman1D_GY, MPU6050.data.Gyro_Y);
      MPU6050.filteredData.Gyro_Z=kalman_filter(&Kalman1D_GZ, MPU6050.data.Gyro_Z);
    }
    if (fail_count >= 3) 
    {
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
      if(MPU6050_Recover(&MPU6050)==HAL_OK)
      {
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
        ret = HAL_OK;
        fail_count = 0;
      }
    }
    tick += pdMS_TO_TICKS(T_ms);
    osDelayUntil(tick);
  }
  /* USER CODE END MPUmeasure */
}

/* USER CODE BEGIN Header_Key */
/**
 * @brief Function implementing the Key_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Key */
void Key(void *argument)
{
  /* USER CODE BEGIN Key */
  uint32_t press_time = 0;
  /* Infinite loop */
  for (;;) {
    // 1. 没按键时，死等在这里睡觉（0% CPU占用）
    if (osSemaphoreAcquire(sem_KeyPressedHandle, osWaitForever) == osOK) {
      osDelay(20);
      if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) != GPIO_PIN_RESET) {
          continue; 
      }
      press_time = osKernelGetTickCount(); // 记录按下的时间
      // 2. 判断按住的时间（这是一个简单的循环，不复杂）
      while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET) {
        if ((osKernelGetTickCount() - press_time) >= 2000) {
          osMutexAcquire(MPU6050_MutexHandle, osWaitForever);
          if(MPU6050_Calibrate(&MPU6050, 500)!=HAL_OK)HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
          osSemaphoreAcquire(sem_KeyPressedHandle, 0);
          osMutexRelease(MPU6050_MutexHandle);
          break;
        }
        osDelay(10);
      }
    }
  }
  /* USER CODE END Key */
}

/* USER CODE BEGIN Header_cpu_monitor */
/**
* @brief Function implementing the cpu_monitor_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_cpu_monitor */
void cpu_monitor(void *argument)
{
  /* USER CODE BEGIN cpu_monitor */
  uint32_t tick = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    tick += pdMS_TO_TICKS(1000);
    osDelayUntil(tick);
    cpu_usage = 100.0f - idle_tick * 100.0f / (float)configTICK_RATE_HZ;
    idle_tick=0;
    ulLastIdleTick = xTaskGetTickCount();
  }
  /* USER CODE END cpu_monitor */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) 
{
  // 1. 识别是不是我们的按键引脚（PE4）
  if (GPIO_Pin == KEY_Pin) {
    // 2. （可选防抖）再读一次电平，确认是真的按下了，而不是毛刺
    if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET) {
      // 3. 释放信号量，通知任务
      // 注意：在中断中调用，不要用带阻塞的API，osSemaphoreRelease是安全的
      osSemaphoreRelease(sem_KeyPressedHandle);
    }
  }
}
/* USER CODE END Application */

