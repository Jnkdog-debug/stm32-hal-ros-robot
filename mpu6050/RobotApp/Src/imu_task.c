#include "imu_task.h"
#include "mpu6050.h"
#include "robot_sys.h"
#include "queue.h"


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
                // 构造消息信封
                TxMessage_t msg;
                msg.type = MSG_TYPE_IMU;
                msg.p_data = p_mpu_data; 
                if (xQueueSend(dataQueueHandle, &msg, 0) != pdPASS) { 
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
}