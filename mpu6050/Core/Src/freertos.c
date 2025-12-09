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
/* 定义系统控制块 */
typedef struct {
    uint8_t is_running;      // 0: 停止, 1: 运行
    uint32_t sample_period;  // 采样周期 (ms)
} SystemControl_t;

/* 初始化默认值: 默认运行，50ms一次 */
SystemControl_t g_sys_ctrl = {1, 50}; 
/* 接收缓冲区 (1字节) */
uint8_t rx_buffer_byte;

/* 解析状态机的状态定义 */
typedef enum {
    STATE_WAIT_HEADER_1,
    STATE_WAIT_HEADER_2,
    STATE_WAIT_CMD,
    STATE_WAIT_PARAM
} ParserState_t;

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
/* Definitions for cmdTask */
osThreadId_t cmdTaskHandle;
const osThreadAttr_t cmdTask_attributes = {
  .name = "cmdTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
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

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartUsartTask(void *argument);
void StartcmdTask(void *argument);

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
  dataQueueHandle = osMessageQueueNew (32, sizeof(MPU6050DATATYPE*), &dataQueue_attributes);

  /* creation of cmdQueue */
  cmdQueueHandle = osMessageQueueNew (32, sizeof(uint8_t), &cmdQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of usartTask */
  usartTaskHandle = osThreadNew(StartUsartTask, NULL, &usartTask_attributes);

  /* creation of cmdTask */
  cmdTaskHandle = osThreadNew(StartcmdTask, NULL, &cmdTask_attributes);

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
  HAL_UART_Receive_IT(&huart3, &rx_buffer_byte, 1);

  MPU6050_Init(Sensor_I2C2_Serch());
  /* Infinite loop */
  for(;;)
    {
        // 1. 读取全局变量中的采样周期
        uint32_t delay_time = g_sys_ctrl.sample_period;
        if (delay_time < 5) delay_time = 5; // 保护一下，别太快卡死

        // 2. 检查是否处于运行状态
        if (g_sys_ctrl.is_running == 1)
        {
            MPU6050DATATYPE* p_mpu_data = (MPU6050DATATYPE*) pvPortMalloc(sizeof(Mpu6050_Data));

            if (p_mpu_data != NULL) 
            {
                MPU6050_Read_Accel();
                MPU6050_Read_Gyro(); 
                MPU6050_Read_Temp();
                *p_mpu_data = Mpu6050_Data; 

                if (xQueueSend(dataQueueHandle, &p_mpu_data, 0) != pdPASS) { 
                    // 这里 timeout 改成 0，如果队列满了就丢弃这帧数据，不要阻塞采集任务
                    vPortFree(p_mpu_data);
                }
            } 
        }
        else
        {
            // 如果停止了，就不申请内存，也不读取 I2C，省电省资源
        }

        // 3. 动态延时
        osDelay(delay_time);
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

/* USER CODE BEGIN Header_StartcmdTask */
/**
* @brief Function implementing the cmdTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartcmdTask */
void StartcmdTask(void *argument)
{
  /* USER CODE BEGIN StartcmdTask */
 
    uint8_t rx_byte;
    uint8_t cmd_cache = 0; // 暂存命令
    ParserState_t current_state = STATE_WAIT_HEADER_1; // 初始状态
  /* Infinite loop */
    for(;;)
    {
        // 从 commandQueue 接收一个字节 (永久阻塞等待)
        if (xQueueReceive(cmdQueueHandle, &rx_byte, portMAX_DELAY) == pdPASS)
        {
            // --- 有限状态机 (FSM) 开始 ---
            switch (current_state)
            {
                case STATE_WAIT_HEADER_1:
                    if (rx_byte == 0xAA) {
                        current_state = STATE_WAIT_HEADER_2;
                    }
                    break;

                case STATE_WAIT_HEADER_2:
                    if (rx_byte == 0x55) {
                        current_state = STATE_WAIT_CMD;
                    } else if (rx_byte == 0xAA) {
                         // 特殊情况：如果是 AA AA，可能第二个是头，保持在 Wait Header 2
                        current_state = STATE_WAIT_HEADER_2; 
                    } else {
                        current_state = STATE_WAIT_HEADER_1; // 错了，重头再来
                    }
                    break;

                case STATE_WAIT_CMD:
                    cmd_cache = rx_byte; // 记下命令
                    current_state = STATE_WAIT_PARAM;
                    break;

                case STATE_WAIT_PARAM:
                    // 收到参数，一帧完整了，开始执行逻辑
                    switch (cmd_cache)
                    {
                        case 0x01: // START
                            g_sys_ctrl.is_running = 1;
                            printf("CMD: System Started\r\n");
                            break;
                            
                        case 0x02: // STOP
                            g_sys_ctrl.is_running = 0;
                            printf("CMD: System Stopped\r\n");
                            break;
                            
                        case 0x03: // SET PERIOD
                            if (rx_byte > 0) {
                                g_sys_ctrl.sample_period = rx_byte * 10; // 参数*10ms
                                printf("CMD: Period set to %lu ms\r\n", g_sys_ctrl.sample_period);
                            }
                            break;
                            
                        default:
                            printf("CMD: Unknown Command\r\n");
                            break;
                    }
                    // 处理完一帧，回到初始状态
                    current_state = STATE_WAIT_HEADER_1;
                    break;
            }
            // --- FSM 结束 ---
        }
    }
  /* USER CODE END StartcmdTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* 串口接收完成回调函数 */
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

