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
#include "mpu6050.h"
#include <stdio.h>
#include "usart.h"
#include "queue.h"
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
/* Definitions for usartTask */
osThreadId_t usartTaskHandle;
const osThreadAttr_t usartTask_attributes = {
  .name = "usartTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for dataQueue */
osMessageQueueId_t dataQueueHandle;
const osMessageQueueAttr_t dataQueue_attributes = {
  .name = "dataQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartUsartTask(void *argument);

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

  /* Create the queue(s) */
  /* creation of dataQueue */
  dataQueueHandle = osMessageQueueNew (16, sizeof(MPU6050DATATYPE*), &dataQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of usartTask */
  usartTaskHandle = osThreadNew(StartUsartTask, NULL, &usartTask_attributes);

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
  MPU6050_Init(Sensor_I2C2_Serch());
  /* Infinite loop */
  for(;;)
    {
        MPU6050DATATYPE* p_mpu_data = (MPU6050DATATYPE*) pvPortMalloc(sizeof(Mpu6050_Data));

        // 1. 检查内存是否分配成功！
        if (p_mpu_data != NULL) 
        {
            // 2. 完整采集数据
            MPU6050_Read_Accel();
            MPU6050_Read_Gyro(); 
            MPU6050_Read_Temp();
            
            // 3. 将全局数据复制到堆内存中
            *p_mpu_data = Mpu6050_Data; 

            // 4. 尝试发送指针
            if (xQueueSend(dataQueueHandle, &p_mpu_data, portMAX_DELAY) != pdPASS) {
                // 发送失败：必须释放内存
                vPortFree(p_mpu_data);
                p_mpu_data = NULL; // 最佳实践
            }
        } 
        else 
        {
            // 内存分配失败，打印警告或报警
            printf("Error: MPU data malloc failed!\r\n");
        }

        // 5. 添加延时以控制采样频率和让出 CPU
        osDelay(50);
    }
  /* USER CODE END StartDefaultTask */
}


/* USER CODE BEGIN Header_StartUsartTask */
/**
* @brief Function implementing the usartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUsartTask */
void StartUsartTask(void *argument)
{
  /* USER CODE BEGIN StartUsartTask */
  /* Infinite loop */
  for(;;)
  {
    MPU6050DATATYPE* p_received_mpu_data; 

    // 1. 接收队列中的**指针**，无限期阻塞等待
    if (xQueueReceive(dataQueueHandle, &p_received_mpu_data, portMAX_DELAY) == pdPASS) {

        // 2. 通过指针访问数据（读取、打印等）
        printf("Temperature: %.2f C\r\n", p_received_mpu_data->Temp);
        printf("Gyro: X=%.2f deg/s, Y=%.2f deg/s, Z=%.2f deg/s\r\n", 
               p_received_mpu_data->Gyro_X, 
               p_received_mpu_data->Gyro_Y, 
               p_received_mpu_data->Gyro_Z);
        printf("Accel: X=%.2f g, Y=%.2f g, Z=%.2f g\r\n", 
               p_received_mpu_data->Accel_X, 
               p_received_mpu_data->Accel_Y, 
               p_received_mpu_data->Accel_Z);
        // 3. **关键步骤：** 使用完毕后，释放生产者分配的内存
        vPortFree(p_received_mpu_data);
        p_received_mpu_data = NULL; 
    }
    //osDelay(100);
  }
  /* USER CODE END StartUsartTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

