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
#include "Motors.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define T_ms 20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern Motor_t motor[4];
/* USER CODE END Variables */
/* Definitions for ReadMotor_Task */
osThreadId_t ReadMotor_TaskHandle;
const osThreadAttr_t ReadMotor_Task_attributes = {
  .name = "ReadMotor_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ChassisTask */
osThreadId_t ChassisTaskHandle;
const osThreadAttr_t ChassisTask_attributes = {
  .name = "ChassisTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SerialParseTask */
osThreadId_t SerialParseTaskHandle;
const osThreadAttr_t SerialParseTask_attributes = {
  .name = "SerialParseTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void ReadMotor(void *argument);
void Chassis(void *argument);
void SerialParse(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ReadMotor_Task */
  ReadMotor_TaskHandle = osThreadNew(ReadMotor, NULL, &ReadMotor_Task_attributes);

  /* creation of ChassisTask */
  ChassisTaskHandle = osThreadNew(Chassis, NULL, &ChassisTask_attributes);

  /* creation of SerialParseTask */
  SerialParseTaskHandle = osThreadNew(SerialParse, NULL, &SerialParseTask_attributes);

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
  /* Infinite loop */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  TickType_t xFrequency = pdMS_TO_TICKS(T_ms);
  for (;;) {
    for (i = 0; i < 4; i++) {
      Encoder_Get(&motor[i]);
      Encoder_to_rpm(&motor[i], T_ms);
      Encoder_to_Speed(&motor[i], T_ms);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
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
    Chassis_Ctrl();
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
  for(;;)
  {
    osDelay(10);
  }
  /* USER CODE END SerialParse */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

