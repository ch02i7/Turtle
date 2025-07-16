/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "chassisR_task.h"
#include "chassisRB_task.h"
#include "chassisL_task.h"
#include "chassisLB_task.h"
#include "connect_task.h"

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

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId CHASSISR_TASKHandle;
osThreadId CHASSISRB_TASKHandle;
osThreadId CHASSISL_TASKHandle;
osThreadId CHASSISLB_TASKHandle;
osThreadId CONNECT_TASKHandle;
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void ChassisR_Task(void const * argument);
void ChassisRB_Task(void const * argument);
void ChassisL_Task(void const * argument);
void ChassisLB_Task(void const * argument);void CONNECT_Task(void const * argument);
extern void MX_USB_DEVICE_Init(void);
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

  /* 创建任务线程 -----------------------------------------------------------*/
  // 默认任务（USB设备初始化）
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);  // 优先级普通，128字堆栈
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  // 右底盘控制任务（高优先级，512字堆栈）
  osThreadDef(CHASSISR_TASK, ChassisR_Task, osPriorityHigh, 0, 512);
  CHASSISR_TASKHandle = osThreadCreate(osThread(CHASSISR_TASK), NULL);

  // 右底盘后驱控制任务（高优先级，512字堆栈）
  osThreadDef(CHASSISRB_TASK, ChassisRB_Task, osPriorityHigh, 0, 512);
  CHASSISRB_TASKHandle = osThreadCreate(osThread(CHASSISRB_TASK), NULL);

  // 左底盘控制任务（高优先级，512字堆栈） 
  osThreadDef(CHASSISL_TASK, ChassisL_Task, osPriorityHigh, 0, 512);
  CHASSISL_TASKHandle = osThreadCreate(osThread(CHASSISL_TASK), NULL);

  // 左底盘后驱控制任务（高优先级，512字堆栈）
  osThreadDef(CHASSISLB_TASK, ChassisLB_Task, osPriorityHigh, 0, 512);
  CHASSISLB_TASKHandle = osThreadCreate(osThread(CHASSISLB_TASK), NULL);

  /* definition and creation of CONNECT_TASK */
  osThreadDef(CONNECT_TASK, CONNECT_Task, osPriorityHigh, 0, 512);
  CONNECT_TASKHandle = osThreadCreate(osThread(CONNECT_TASK), NULL);


  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_ChassisR_Task */
/**
* @brief Function implementing the CHASSISR_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ChassisR_Task */
void ChassisR_Task(void const * argument)
{
  /* USER CODE BEGIN ChassisR_Task */
  /* Infinite loop */
  for(;;)
  {
    ChassisR_task();
  }
  /* USER CODE END ChassisR_Task */
}

/* USER CODE BEGIN Header_ChassisRB_Task */
/**
* @brief Function implementing the CHASSISRB_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ChassisRB_Task */
void ChassisRB_Task(void const * argument)
{
  /* USER CODE BEGIN ChassisRB_Task */
  /* Infinite loop */
  for(;;)
  {
    ChassisRB_task();
  }
  /* USER CODE END ChassisRB_Task */
}

/* USER CODE BEGIN Header_ChassisL_Task */
/**
* @brief Function implementing the CHASSISL_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ChassisL_Task */
void ChassisL_Task(void const * argument)
{
  /* USER CODE BEGIN ChassisL_Task */
  /* Infinite loop */
  for(;;)
  {
    ChassisL_task();
  }
  /* USER CODE END ChassisL_Task */
}

/* USER CODE BEGIN Header_ChassisLB_Task */
/**
* @brief Function implementing the CHASSISLB_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ChassisLB_Task */
void ChassisLB_Task(void const * argument)
{
  /* USER CODE BEGIN ChassisLB_Task */
  /* Infinite loop */
  for(;;)
  {
    ChassisLB_task();
  }
  /* USER CODE END ChassisLB_Task */
}
void CONNECT_Task(void const * argument)
{
  /* USER CODE BEGIN CONNECT_Task */
  /* Infinite loop */
  for(;;)
  {
    Connect_task();
  }
  /* USER CODE END CONNECT_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
