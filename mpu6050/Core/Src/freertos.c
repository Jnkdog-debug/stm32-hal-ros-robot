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
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include "usart.h"
#include "queue.h"
#include "robot_sys.h"
#include "mpu6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/**
 * @brief 重写 Newlib 的 _write 函数，实现 printf 重定向
 * @param file: 文件描述符 (1=stdout, 2=stderr)
 * @param data: 待发送数据缓冲区
 * @param len: 待发送数据长度
 * @retval 实际写入的字节数
 */
int _write(int file, char *data, int len)

{

HAL_UART_Transmit(&huart3, (uint8_t*)data, len, HAL_MAX_DELAY);

return len;

}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 定义系统控制块 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

// 全局变量的定义

/* 初始化默认值: 默认运行，50ms一次 */
Robot_State_t g_robot = {1, 50};
/* 接收缓冲区 (1字节) */




/* USER CODE END Variables */
/* Definitions for imuTask */
osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
  .name = "imuTask",
  .stack_size = 600 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for usartTask */
osThreadId_t usartTaskHandle;
const osThreadAttr_t usartTask_attributes = {
  .name = "usartTask",
  .stack_size = 600 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for cmdTask */
osThreadId_t cmdTaskHandle;
const osThreadAttr_t cmdTask_attributes = {
  .name = "cmdTask",
  .stack_size = 600 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for motorTask */
osThreadId_t motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
  .name = "motorTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for dataQueue */
osMessageQueueId_t dataQueueHandle;
const osMessageQueueAttr_t dataQueue_attributes = {
  .name = "dataQueue"
};
/* Definitions for cmdQueue */
osMessageQueueId_t cmdQueueHandle;
const osMessageQueueAttr_t cmdQueue_attributes = {
  .name = "cmdQueue"
};
/* Definitions for robotMutex */
osMutexId_t robotMutexHandle;
const osMutexAttr_t robotMutex_attributes = {
  .name = "robotMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartimuTask(void *argument);
void StartUsartTask(void *argument);
void StartcmdTask(void *argument);
void StartmotorTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of robotMutex */
  robotMutexHandle = osMutexNew(&robotMutex_attributes);

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
  /* creation of dataQueue */
  dataQueueHandle = osMessageQueueNew (32, sizeof(TxMessage_t), &dataQueue_attributes);

  /* creation of cmdQueue */
  cmdQueueHandle = osMessageQueueNew (32, sizeof(uint8_t), &cmdQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of imuTask */
  imuTaskHandle = osThreadNew(StartimuTask, NULL, &imuTask_attributes);

  /* creation of usartTask */
  usartTaskHandle = osThreadNew(StartUsartTask, NULL, &usartTask_attributes);

  /* creation of cmdTask */
  cmdTaskHandle = osThreadNew(StartcmdTask, NULL, &cmdTask_attributes);

  /* creation of motorTask */
  motorTaskHandle = osThreadNew(StartmotorTask, NULL, &motorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartimuTask */
/**
  * @brief  Function implementing the imuTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartimuTask */
__weak void StartimuTask(void *argument)
{
  /* USER CODE BEGIN StartimuTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartimuTask */
}

/* USER CODE BEGIN Header_StartUsartTask */
/**
* @brief Function implementing the usartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUsartTask */
__weak void StartUsartTask(void *argument)
{
  /* USER CODE BEGIN StartUsartTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartUsartTask */
}

/* USER CODE BEGIN Header_StartcmdTask */
/**
* @brief Function implementing the cmdTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartcmdTask */
__weak void StartcmdTask(void *argument)
{
  /* USER CODE BEGIN StartcmdTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartcmdTask */
}

/* USER CODE BEGIN Header_StartmotorTask */
/**
* @brief Function implementing the motorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartmotorTask */
__weak void StartmotorTask(void *argument)
{
  /* USER CODE BEGIN StartmotorTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartmotorTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* 串口接收完成回调函数 */
uint8_t rx_buffer_byte;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) // 确认是哪个串口
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // 把收到的这 1 个字节发送到 commandQueue
        // 注意：这是在中断里，要用 FromISR 版本
        xQueueSendFromISR(cmdQueueHandle, &rx_buffer_byte, &xHigherPriorityTaskWoken);

        // 重新开启中断接收下一个字节
        HAL_UART_Receive_IT(&huart3, &rx_buffer_byte, 1);

        // 如果唤醒了高优先级任务，进行上下文切换
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
/* USER CODE END Application */

