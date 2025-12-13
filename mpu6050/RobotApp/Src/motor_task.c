
#include "motor_task.h"
#include "robot_sys.h"
#include "encodermotor.h"
#include "queue.h"
#include "cmsis_os.h" // 确保包含 RTOS 头文件

// 引入外部定义的电机实例
extern MotorControl_t Motor_A;

extern MotorControl_t Motor_D;

// 10ms 控制周期 (100Hz)
#define MOTOR_CTRL_PERIOD_MS 10



void StartmotorTask(void *argument)
{
    // --- 1. 初始化阶段 ---
    // 确保 Motor_Encoder_Init 函数已包含 HAL_TIM_Encoder_Start()
    Motor_Init(&Motor_A);
    Motor_Init(&Motor_D); 

    Motor_Encoder_Init(&Motor_A);
    Motor_Encoder_Init(&Motor_D);



    uint8_t debug_counter = 0;
    
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    float speed_L = 0.0f;
    float speed_R = 0.0f;

    int32_t encoder_count_L = 0;
    int32_t encoder_count_R = 0;

    for(;;)
    {
        // 从全局命令中读取目标速度（上位机通过 cmdTask 写入 g_robot.speed_*）
        osMutexAcquire(robotMutexHandle, osWaitForever);
        {
            speed_L = g_robot.speed_L;
            speed_R = g_robot.speed_R;
        }
        osMutexRelease(robotMutexHandle);
        
        
        
    
        Motor_SetPWM(&Motor_A, speed_L);
        Motor_SetPWM(&Motor_D, speed_R);
        encoder_count_L = Motor_Encoder_Read(&Motor_A);
        encoder_count_R = Motor_Encoder_Read(&Motor_D);

        // 更新全局编码器值，供其他任务使用（例如上位机查询或日志）
        osMutexAcquire(robotMutexHandle, osWaitForever);
        {
            g_robot.encoder_L = encoder_count_L;
            g_robot.encoder_R = encoder_count_R;
        }
        osMutexRelease(robotMutexHandle);
        
        
        
        if (++debug_counter >= 10) 
        {
            debug_counter = 0;

            MotorData_t* p_data = pvPortMalloc(sizeof(MotorData_t));
            if (p_data != NULL)
            {
                // 这里使用编码器计数作为反馈（以 float 形式发送）
                p_data->speed_L = encoder_count_L;
                p_data->speed_R = encoder_count_R;
                
                TxMessage_t msg;
                msg.type = MSG_TYPE_MOTOR;
                msg.p_data = p_data;
                
                if (xQueueSend(dataQueueHandle, &msg, 0) != pdPASS) {
                    vPortFree(p_data);
                }
            }
         }

        // --- 6. 精准控制周期 ---
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MOTOR_CTRL_PERIOD_MS));
    }
}