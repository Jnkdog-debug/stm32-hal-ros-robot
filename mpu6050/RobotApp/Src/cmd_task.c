#include "cmd_task.h"
#include "robot_sys.h"
#include "stdio.h" 
#include <string.h> // 用于 memcpy 

//命令解析函数
void Process_Command(uint8_t func, uint8_t* payload, uint8_t len)
{
    // 获取互斥锁，因为我们要修改全局变量了
    osMutexAcquire(robotMutexHandle, osWaitForever);

    switch (func)
    {
        case 0x01: // 启停
            g_robot.is_running = payload[0];
            break;

        case 0x10: // 设置目标速度 (2个 float = 8字节)
            if (len == 8) {
                // 使用 memcpy 安全地转换类型
                memcpy(&g_robot.target_speed_L, &payload[0], 4);
                memcpy(&g_robot.target_speed_R, &payload[4], 4);
            }
            break;

        case 0x11: // 设置 PID (3个 float = 12字节)
            if (len == 12) {
                memcpy(&g_robot.pid_vel_kp, &payload[0], 4);
                memcpy(&g_robot.pid_vel_ki, &payload[4], 4);
                memcpy(&g_robot.pid_vel_kd, &payload[8], 4);
            }
            break;
            
        // ... 更多命令
    }

    // 释放锁
    osMutexRelease(robotMutexHandle);
}




void StartcmdTask(void *argument)
{
    uint8_t rx_byte;
    FrameState_t state = STATE_WAIT_HEADER_1;
    
    // 临时变量用来存一帧数据
    uint8_t frame_func = 0;
    uint8_t frame_len = 0;
    uint8_t payload_buf[MAX_PAYLOAD_LEN];
    uint8_t payload_count = 0;
    uint8_t calc_crc = 0; // 用来算校验和

    for(;;)
    {
        if (xQueueReceive(cmdQueueHandle, &rx_byte, portMAX_DELAY) == pdPASS)
        {
            switch (state)
            {
                case STATE_WAIT_HEADER_1:
                    if (rx_byte == 0xAA) state = STATE_WAIT_HEADER_2;
                    break;

                case STATE_WAIT_HEADER_2:
                    if (rx_byte == 0x55) {
                        state = STATE_WAIT_FUNC;
                        calc_crc = 0; // 重置校验和计算
                    } else if (rx_byte == 0xAA) {
                        state = STATE_WAIT_HEADER_2;
                    } else {
                        state = STATE_WAIT_HEADER_1;
                    }
                    break;

                case STATE_WAIT_FUNC:
                    frame_func = rx_byte;
                    calc_crc += rx_byte; // 累加校验
                    state = STATE_WAIT_LEN;
                    break;

                case STATE_WAIT_LEN:
                    frame_len = rx_byte;
                    calc_crc += rx_byte;
                    if (frame_len == 0) {
                        state = STATE_WAIT_CRC; // 没数据，直接跳去校验
                    } else if (frame_len > MAX_PAYLOAD_LEN) {
                        state = STATE_WAIT_HEADER_1; // 长度非法，报错重置
                    } else {
                        payload_count = 0;
                        state = STATE_WAIT_PAYLOAD;
                    }
                    break;

                case STATE_WAIT_PAYLOAD:
                    payload_buf[payload_count++] = rx_byte;
                    calc_crc += rx_byte;
                    if (payload_count == frame_len) {
                        state = STATE_WAIT_CRC;
                    }
                    break;

                case STATE_WAIT_CRC:
                    if (calc_crc == rx_byte) {
                        // --- 校验通过！执行命令解析 ---
                        Process_Command(frame_func, payload_buf, frame_len);
                    } else {
                        printf("Error: CRC Failed!\r\n");
                    }
                    state = STATE_WAIT_HEADER_1; // 回到起点
                    break;
            }
        }
        osDelay(100);

    }
}