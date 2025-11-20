#include "MPU6050_task.h"
#include "cmsis_os.h"
#include "protocol.h"
#include "uart_tasks.h" // 引用发送队列句柄
#include "Mpu6050.h" // 假设包含您的编码器和电机驱动





/**
 * @brief Function implementing the MPU6050Task thread.
 */
void StartMPU6050Task(void *argument)
{
    /* Infinite loop */
    for(;;)
    {
        // MPU6050 传感器读取逻辑
        osDelay(100);
    }
}