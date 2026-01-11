#include "usart_task.h"
#include "robot_sys.h"
#include "mpu6050.h"
#include "usart.h"
#include <string.h> // 需要 memcpy
//这是发送任务
/* 辅助函数：计算简单累加和校验 */
uint8_t Calc_Checksum(uint8_t *data, uint8_t len) {
    uint8_t sum = 0;
    for (int i = 0; i < len; i++) sum += data[i];
    return sum;
}

/* 串口发送任务入口 */
void StartUsartTask(void *argument)
{
    TxMessage_t recv_msg; // 接收到的通用信封
    uint8_t tx_buf[64];   // 发送缓冲区
    
    for(;;)
    {
        // 1. 等待信封
        if (xQueueReceive(dataQueueHandle, &recv_msg, portMAX_DELAY) == pdPASS) {
            
            // --- 公共帧头 ---
            tx_buf[0] = 0xAA;
            tx_buf[1] = 0x55;
            
            uint8_t payload_len = 0;
            
            // 2. 根据类型分拣
            switch (recv_msg.type)
            {
                case MSG_TYPE_IMU:
                {
                    // 把 void* 强转回 IMU 指针
                    MPU6050DATATYPE* p_imu = (MPU6050DATATYPE*)recv_msg.p_data;
                    
                    tx_buf[2] = 0x10; // 功能码: IMU
                    
                    // 【修改点 1】长度改为 24 (6个 float * 4字节)
                    // 原来只发加速度是 12，现在加上角速度是 24
                    tx_buf[3] = 24;   
                    
                    // --- 打包加速度 (Accel) ---
                    memcpy(&tx_buf[4],  &p_imu->Accel_X, 4);
                    memcpy(&tx_buf[8],  &p_imu->Accel_Y, 4);
                    memcpy(&tx_buf[12], &p_imu->Accel_Z, 4);
                    
                    // --- 【修改点 2】打包角速度 (Gyro) ---
                    // 紧接着加速度后面存放
                    memcpy(&tx_buf[16], &p_imu->Gyro_X, 4);
                    memcpy(&tx_buf[20], &p_imu->Gyro_Y, 4);
                    memcpy(&tx_buf[24], &p_imu->Gyro_Z, 4);
                    
                    payload_len = 24;
                    break;
                }
                
                case MSG_TYPE_MOTOR:
                {
                    MotorData_t* p_motor = (MotorData_t*)recv_msg.p_data;
                    
                    tx_buf[2] = 0x20; // 功能码
                    
                    // 【修改点1】长度改为 16 字节 (4个变量 * 4字节)
                    tx_buf[3] = 16;   
                    
                    // 【修改点2】依次打包 4 个数据
                    // 偏移 4: 左脉冲 (int)
                    memcpy(&tx_buf[4],  &p_motor->count_L, 4);
                    // 偏移 8: 右脉冲 (int)
                    memcpy(&tx_buf[8],  &p_motor->count_R, 4);
                    // 偏移 12: 左速度 (float)
                    memcpy(&tx_buf[12], &p_motor->speed_L,   4);
                    // 偏移 16: 右速度 (float)
                    memcpy(&tx_buf[16], &p_motor->speed_R,   4);
                    
                    payload_len = 16;
                    break;
                }
                
                default:
                    break;
            }

            // 3. 计算校验并发送
            if (payload_len > 0) {
                // 校验位放在 payload 后面
                // 总长度 = 头(2) + 功能(1) + 长度(1) + Payload + 校验(1)
                uint8_t total_len = 2 + 1 + 1 + payload_len + 1;
                
                // 计算前面所有字节的校验和 (除了最后一个字节)
                tx_buf[total_len - 1] = Calc_Checksum(&tx_buf[2], total_len - 3);                
                // 发送二进制流
                HAL_UART_Transmit(&huart3, tx_buf, total_len, 10);
            }

            // 4. ***最重要的一步***：释放内存
            // 无论是什么类型，recv_msg.p_data 指向的都是 malloc 出来的内存
            vPortFree(recv_msg.p_data);
        }
    }
}