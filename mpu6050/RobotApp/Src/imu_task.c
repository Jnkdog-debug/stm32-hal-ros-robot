#include "imu_task.h"
#include "mpu6050.h"
#include "robot_sys.h"
#include "queue.h"
#include <stdio.h>

// --- 定义转换系数 ---
// 1 g = 9.80665 m/s^2
#define G_TO_MSS    9.80665f
// 1 degree = 0.017453 radians
#define DEG_TO_RAD  0.0174532925f

void StartimuTask(void *argument)
{
    /* USER CODE BEGIN StartDefaultTask */
    HAL_UART_Receive_IT(&huart3, &rx_buffer_byte, 1);

    // 1. 初始化 IMU
    MPU6050_Init(Sensor_I2C2_Serch());
    osDelay(200); // 等待传感器内部电路稳定

    // ==========================================
    // 2. 上电自动校准 (消除静止时的漂移)
    // ==========================================
    float gyro_x_offset = 0.0f, gyro_y_offset = 0.0f, gyro_z_offset = 0.0f;
    const int CALIB_COUNT = 300; // 采样 300 次求平均

    // 提示：在此期间请保持机器人静止！
    for(int i = 0; i < CALIB_COUNT; i++) 
    {
        MPU6050_Read_Gyro();
        // 累加误差 (此时读出来的是 deg/s)
        gyro_x_offset += Mpu6050_Data.Gyro_X;
        gyro_y_offset += Mpu6050_Data.Gyro_Y;
        gyro_z_offset += Mpu6050_Data.Gyro_Z;
        osDelay(2); // 稍微延时
    }
    
    // 计算平均误差
    gyro_x_offset /= (float)CALIB_COUNT;
    gyro_y_offset /= (float)CALIB_COUNT;
    gyro_z_offset /= (float)CALIB_COUNT;
    
    // ==========================================
    // 3. 开始循环采集
    // ==========================================
    for(;;)
    {
        // 读取采样周期设置
        uint32_t delay_time = g_robot.sample_period;
        if (delay_time < 5) delay_time = 5; 

        if (g_robot.is_running == 1)
        {
            MPU6050DATATYPE* p_mpu_data = (MPU6050DATATYPE*) pvPortMalloc(sizeof(Mpu6050_Data));

            if (p_mpu_data != NULL) 
            {
                // A. 读取原始数据 (单位: g, deg/s)
                MPU6050_Read_Accel();
                MPU6050_Read_Gyro(); 
                // MPU6050_Read_Temp(); // 温度不是必须的

                // B. 处理角速度 (减去零偏 + 转弧度)
                // 1. 减去上电时的静态误差
                float correct_gx = Mpu6050_Data.Gyro_X - gyro_x_offset;
                float correct_gy = Mpu6050_Data.Gyro_Y - gyro_y_offset;
                float correct_gz = Mpu6050_Data.Gyro_Z - gyro_z_offset;

                // 2. 转换为 rad/s (ROS 标准)
                p_mpu_data->Gyro_X = correct_gx * DEG_TO_RAD;
                p_mpu_data->Gyro_Y = correct_gy * DEG_TO_RAD;
                p_mpu_data->Gyro_Z = correct_gz * DEG_TO_RAD;

                // C. 处理加速度 (转 m/s^2)
                // 加速度计通常不需要减 offset，因为重力是一直存在的
                p_mpu_data->Accel_X = Mpu6050_Data.Accel_X * G_TO_MSS;
                p_mpu_data->Accel_Y = Mpu6050_Data.Accel_Y * G_TO_MSS;
                p_mpu_data->Accel_Z = Mpu6050_Data.Accel_Z * G_TO_MSS;

                // D. 发送数据
                TxMessage_t msg;
                msg.type = MSG_TYPE_IMU;
                msg.p_data = p_mpu_data; 
                
                if (xQueueSend(dataQueueHandle, &msg, 0) != pdPASS) { 
                    vPortFree(p_mpu_data);
                }
            } 
        }
        
        osDelay(delay_time);
    }
}