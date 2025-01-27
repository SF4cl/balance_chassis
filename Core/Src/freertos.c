/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "bsp_dwt.h"
#include "cmsis_os2.h"
#include "stm32h723xx.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
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
uint32_t pwm_out = 0;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for imu_task */
osThreadId_t imu_taskHandle;
uint32_t imu_taskBuffer[ 256 ];
osStaticThreadDef_t imu_taskControlBlock;
const osThreadAttr_t imu_task_attributes = {
  .name = "imu_task",
  .cb_mem = &imu_taskControlBlock,
  .cb_size = sizeof(imu_taskControlBlock),
  .stack_mem = &imu_taskBuffer[0],
  .stack_size = sizeof(imu_taskBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for chassis_task */
osThreadId_t chassis_taskHandle;
uint32_t chassis_taskBuffer[ 512 ];
osStaticThreadDef_t chassis_taskControlBlock;
const osThreadAttr_t chassis_task_attributes = {
  .name = "chassis_task",
  .cb_mem = &chassis_taskControlBlock,
  .cb_size = sizeof(chassis_taskControlBlock),
  .stack_mem = &chassis_taskBuffer[0],
  .stack_size = sizeof(chassis_taskBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for remote_task */
osThreadId_t remote_taskHandle;
uint32_t remote_taskBuffer[ 256 ];
osStaticThreadDef_t remote_taskControlBlock;
const osThreadAttr_t remote_task_attributes = {
  .name = "remote_task",
  .cb_mem = &remote_taskControlBlock,
  .cb_size = sizeof(remote_taskControlBlock),
  .stack_mem = &remote_taskBuffer[0],
  .stack_size = sizeof(remote_taskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for imu_data */
osMessageQueueId_t imu_dataHandle;
const osMessageQueueAttr_t imu_data_attributes = {
  .name = "imu_data"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void imu_task_entry(void *argument);
void chassis_task_entry(void *argument);
void remote_task_entry(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

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
  /* creation of imu_data */
  imu_dataHandle = osMessageQueueNew (16, sizeof(uint16_t), &imu_data_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of imu_task */
  imu_taskHandle = osThreadNew(imu_task_entry, NULL, &imu_task_attributes);

  /* creation of chassis_task */
  chassis_taskHandle = osThreadNew(chassis_task_entry, NULL, &chassis_task_attributes);

  /* creation of remote_task */
  remote_taskHandle = osThreadNew(remote_task_entry, NULL, &remote_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_imu_task_entry */
/**
* @brief Function implementing the imu_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_imu_task_entry */
void imu_task_entry(void *argument)
{
  /* USER CODE BEGIN imu_task_entry */
  static float imu_start;
  static float imu_dt;

  ImuInit();
  /* Infinite loop */
  for(;;)
  {
    imu_start = DWT_GetTimeline_ms();

    ImuUpdate();
    pwm_out = GetPwmOut();
    htim3.Instance->CCR4 = pwm_out;

    vTaskDelay(1);

    imu_dt = DWT_GetTimeline_ms() - imu_start;
  }
  /* USER CODE END imu_task_entry */
}

/* USER CODE BEGIN Header_chassis_task_entry */
/**
* @brief Function implementing the chassis_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chassis_task_entry */
void chassis_task_entry(void *argument)
{
  /* USER CODE BEGIN chassis_task_entry */
  static float chassis_start;
  static float chassis_dt;

  ChassisInit();
  /* Infinite loop */
  for(;;)
  {
    chassis_start = DWT_GetTimeline_ms();

    // vTaskSuspendAll();
    
    ChassisUpdate();
    
    // xTaskResumeAll();

    chassis_dt = DWT_GetTimeline_ms() - chassis_start;

    vTaskDelay(2);

    // chassis_dt = DWT_GetTimeline_ms() - chassis_start;
  }
  /* USER CODE END chassis_task_entry */
}

/* USER CODE BEGIN Header_remote_task_entry */
/**
* @brief Function implementing the remote_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_remote_task_entry */
void remote_task_entry(void *argument)
{
  /* USER CODE BEGIN remote_task_entry */

  /* Infinite loop */
  for(;;)
  {
    vTaskDelay(2);
  }
  /* USER CODE END remote_task_entry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

