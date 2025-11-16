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
#include "queue.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "usart.h"
#include "i2c.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int _write(int file, char *data, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)data, len, HAL_MAX_DELAY);
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
/* Definitions for ProcessingTask */
osThreadId_t ProcessingTaskHandle;
const osThreadAttr_t ProcessingTask_attributes = {
  .name = "ProcessingTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UartTask */
osThreadId_t UartTaskHandle;
const osThreadAttr_t UartTask_attributes = {
  .name = "UartTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for dataQueue01 */
osMessageQueueId_t dataQueue01Handle;
const osMessageQueueAttr_t dataQueue01_attributes = {
  .name = "dataQueue01"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartProcessingTask(void *argument);
void StartSensorTask(void *argument);
void StartUartTask(void *argument);

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
  /* creation of dataQueue01 */
  dataQueue01Handle = osMessageQueueNew (100, sizeof(MPU6050DATATYPE*), &dataQueue01_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ProcessingTask */
  ProcessingTaskHandle = osThreadNew(StartProcessingTask, NULL, &ProcessingTask_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StartSensorTask, NULL, &SensorTask_attributes);

  /* creation of UartTask */
  UartTaskHandle = osThreadNew(StartUartTask, NULL, &UartTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartProcessingTask */
/**
  * @brief  Function implementing the ProcessingTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartProcessingTask */
void StartProcessingTask(void *argument)
{
  /* USER CODE BEGIN StartProcessingTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartProcessingTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN StartSensorTask */
  MPU6050_Init(Sensor_I2C2_Serch());


  /* Infinite loop */
  for(;;)
  {

  // 队列发送的是指针，是结构体的指针，这比直接复制数据再传输要快
MPU6050DATATYPE* p_mpu_data = (MPU6050DATATYPE*) pvPortMalloc(sizeof(Mpu6050_Data));
      

//1 检查内存是否充足
      if (p_mpu_data == NULL) {
            // 内存不足，处理错误，例如延时后重试
            osDelay(100);
            continue;
        }


      MPU6050_Read_Accel();
      MPU6050_Read_Gyro();
      MPU6050_Read_Temp();


        *p_mpu_data = Mpu6050_Data; // 结构体整体赋值，将全局数据复制到堆内存

 if (xQueueSend(dataQueue01Handle, &p_mpu_data, portMAX_DELAY) != pdPASS) {
        // 如果发送失败（例如队列已满），则需要释放已分配的内存，否则会造成内存泄漏
        vPortFree(p_mpu_data);
        p_mpu_data = NULL; // 最佳实践：将指针置空
        // 可以添加错误日志或告警
    }


     osDelay(100);
  } 

  /* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartUartTask */
/**
* @brief Function implementing the UartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartTask */
void StartUartTask(void *argument)
{
  /* USER CODE BEGIN StartUartTask */
   MPU6050DATATYPE* p_received_mpu_data; // 定义一个指针变量，用于接收从队列中取出的数据地址

  /* Infinite loop */
  for(;;)
  {
    // 1. 从队列接收指向 MPU6050_Data_t 结构体的指针
    // xQueueReceive 的第二个参数是一个指向指针的指针，因为队列会将接收到的指针值（地址）写入 p_received_mpu_data
    if (xQueueReceive(dataQueue01Handle, &p_received_mpu_data, portMAX_DELAY) == pdPASS) {
      // 成功从队列中接收到了数据指针

      // 2. 通过接收到的指针 p_received_mpu_data 访问 MPU6050_Data_t 结构体中的各个成员
      // 并使用 printf 将数据格式化输出到串口
      printf("--- MPU6050 Data ---\r\n"); // \r\n 用于新行和回车，确保在串口终端显示正确
      printf("Temperature: %.2f C\r\n", p_received_mpu_data->Temp);
      printf("Accel (X, Y, Z): %.2f, %.2f, %.2f\r\n",
             p_received_mpu_data->Accel_X,
             p_received_mpu_data->Accel_Y,
             p_received_mpu_data->Accel_Z);
      printf("Gyro (X, Y, Z): %.2f, %.2f, %.2f\r\n",
             p_received_mpu_data->Gyro_X,
             p_received_mpu_data->Gyro_Y,
             p_received_mpu_data->Gyro_Z);
      printf("--------------------\r\n\r\n");

      // 3. **重要：** 使用完毕后，**释放**动态分配的内存
      // 这块内存是 SensorTask 中通过 pvPortMalloc 分配的，必须由接收方释放，否则会造成内存泄漏。
      vPortFree(p_received_mpu_data);
      p_received_mpu_data = NULL; // 将指针置空，避免后续意外使用已释放的内存（野指针）
    }

    osDelay(50); // 短暂延时，避免任务空转占用过多CPU，让其他任务有机会运行
  /* USER CODE END StartUartTask */
}
}
/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

