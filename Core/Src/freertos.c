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
#include "cmsis_os.h"
#include "main.h"
#include "task.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU6050.h"
#include "Motors.h"
#include <stdint.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern Motor_t motor[4];
extern MPU6050_t MPU6050;
/* USER CODE END Variables */
/* Definitions for ReadMotor_Task */
osThreadId_t ReadMotor_TaskHandle;
const osThreadAttr_t ReadMotor_Task_attributes = {
    .name = "ReadMotor_Task",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for Chassis_Task */
osThreadId_t Chassis_TaskHandle;
const osThreadAttr_t Chassis_Task_attributes = {
    .name = "Chassis_Task",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for SerialParse_Task */
osThreadId_t SerialParse_TaskHandle;
const osThreadAttr_t SerialParse_Task_attributes = {
    .name = "SerialParse_Task",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for IMU_Task */
osThreadId_t IMU_TaskHandle;
const osThreadAttr_t IMU_Task_attributes = {
    .name = "IMU_Task",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for SerialData */
osMessageQueueId_t SerialDataHandle;
const osMessageQueueAttr_t SerialData_attributes = {.name = "SerialData"};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void ReadMotor(void *argument);
void Chassis(void *argument);
void SerialParse(void *argument);
void IMU(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook(void) {
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

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of SerialData */
  SerialDataHandle =
      osMessageQueueNew(1024, sizeof(uint8_t), &SerialData_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ReadMotor_Task */
  ReadMotor_TaskHandle =
      osThreadNew(ReadMotor, NULL, &ReadMotor_Task_attributes);

  /* creation of Chassis_Task */
  Chassis_TaskHandle = osThreadNew(Chassis, NULL, &Chassis_Task_attributes);

  /* creation of SerialParse_Task */
  SerialParse_TaskHandle =
      osThreadNew(SerialParse, NULL, &SerialParse_Task_attributes);

  /* creation of IMU_Task */
  IMU_TaskHandle = osThreadNew(IMU, NULL, &IMU_Task_attributes);

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
void ReadMotor(void *argument) {
  /* USER CODE BEGIN ReadMotor */
  Motors_Init();
  uint8_t i = 0;
  uint32_t tick = osKernelGetTickCount();
  uint8_t T_ms=20;
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
void Chassis(void *argument) {
  /* USER CODE BEGIN Chassis */
  /* Infinite loop */
  for (;;) {
    //Chassis_Ctrl();
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
void SerialParse(void *argument) {
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
void IMU(void *argument) {
  /* USER CODE BEGIN IMU */
  MPU6050_Init();
  MPU6050_GetID(&MPU6050);
  uint32_t tick = osKernelGetTickCount();
  uint8_t T_ms=1000/MPU6050.config.Sample_Rate;
  /* Infinite loop */
  for (;;) {
    MPU6050_ReadAccX(&MPU6050);
    MPU6050_ReadAccY(&MPU6050);
    MPU6050_ReadAccZ(&MPU6050);
    MPU6050_ReadGyroX(&MPU6050);
    MPU6050_ReadGyroY(&MPU6050);
    MPU6050_ReadGyroZ(&MPU6050);
    MPU6050_ReadTemp(&MPU6050);
    tick += pdMS_TO_TICKS(T_ms);
    osDelayUntil(tick);
  }
  /* USER CODE END IMU */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
