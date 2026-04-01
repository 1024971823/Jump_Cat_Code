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
#include "tsk_config_and_callback.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ctrl_task */
osThreadId_t ctrl_taskHandle;
uint32_t ctrl_taskBuffer[ 2048 ];
osStaticThreadDef_t ctrl_taskControlBlock;
const osThreadAttr_t ctrl_task_attributes = {
  .name = "ctrl_task",
  .cb_mem = &ctrl_taskControlBlock,
  .cb_size = sizeof(ctrl_taskControlBlock),
  .stack_mem = &ctrl_taskBuffer[0],
  .stack_size = sizeof(ctrl_taskBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for remote_task */
osThreadId_t remote_taskHandle;
uint32_t remote_taskBuffer[ 1024 ];
osStaticThreadDef_t remote_taskControlBlock;
const osThreadAttr_t remote_task_attributes = {
  .name = "remote_task",
  .cb_mem = &remote_taskControlBlock,
  .cb_size = sizeof(remote_taskControlBlock),
  .stack_mem = &remote_taskBuffer[0],
  .stack_size = sizeof(remote_taskBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for monitor_task */
osThreadId_t monitor_taskHandle;
uint32_t monitor_taskBuffer[ 512 ];   /* 原128字不足以运行监控输出, 扩至512字(2KB) */
osStaticThreadDef_t monitor_taskControlBlock;
const osThreadAttr_t monitor_task_attributes = {
  .name = "monitor_task",
  .cb_mem = &monitor_taskControlBlock,
  .cb_size = sizeof(monitor_taskControlBlock),
  .stack_mem = &monitor_taskBuffer[0],
  .stack_size = sizeof(monitor_taskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Start_ctrl_task(void *argument);
void Start_remote_task(void *argument);
void Start_monitor_task(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
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

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ctrl_task */
  ctrl_taskHandle = osThreadNew(Start_ctrl_task, NULL, &ctrl_task_attributes);

  /* creation of remote_task */
  remote_taskHandle = osThreadNew(Start_remote_task, NULL, &remote_task_attributes);

  /* creation of monitor_task */
  monitor_taskHandle = osThreadNew(Start_monitor_task, NULL, &monitor_task_attributes);

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

/* USER CODE BEGIN Header_Start_ctrl_task */
/**
* @brief Function implementing the ctrl_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_ctrl_task */
void Start_ctrl_task(void *argument)
{
  /* USER CODE BEGIN Start_ctrl_task */
  /* 使用 osDelayUntil 保证严格 1ms 周期, 不受任务体执行时间漂移 */
  uint32_t tick = osKernelGetTickCount();
  for(;;)
  {
    tick += 1U;
    osDelayUntil(tick);
    RTOS_Ctrl_Task_Loop();
  }
  /* USER CODE END Start_ctrl_task */
}

/* USER CODE BEGIN Header_Start_remote_task */
/**
* @brief Function implementing the remote_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_remote_task */
void Start_remote_task(void *argument)
{
  /* USER CODE BEGIN Start_remote_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_remote_task */
}

/* USER CODE BEGIN Header_Start_monitor_task */
/**
* @brief Function implementing the monitor_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_monitor_task */
void Start_monitor_task(void *argument)
{
  /* USER CODE BEGIN Start_monitor_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_monitor_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

