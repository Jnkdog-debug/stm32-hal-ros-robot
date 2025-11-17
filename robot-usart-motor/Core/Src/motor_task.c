#include "motor_task.h"
#include "cmsis_os.h"
#include "protocol.h"
#include "uart_tasks.h" // 引用发送队列句柄
#include "encodermotor.h" // 假设包含您的编码器和电机驱动

// 引用 CubeMX 自动生成的发送队列句柄
extern osMessageQueueId_t UartTxQueueHandle;

// 引用 CubeMX 自动生成的任务句柄（在 freertos.c 中定义）
extern osThreadId_t MotorTaskHandle; 

// 模拟编码器读取函数
// 在实际项目中，这会读取 Timer/Encoder 寄存器
uint32_t ENCODER_Read_Value_Simulate(void) {
    // 假设每 100ms 计数值增加 1000
    static uint32_t counter = 0;
    counter += 1000;
    return counter;
}

static void MotorTask_SendReport(void) {
    uint32_t count = ENCODER_Read_Value_Simulate();
    protocol_frame_t tx_frame;
    
    if (Protocol_PackFrame(&tx_frame, CMD_ID_ENCODER_REPORT, (uint8_t*)&count, sizeof(count)) == PROTOCOL_OK) {
        // 使用 osWaitForever 确保关键数据能进入队列
        osMessageQueuePut(UartTxQueueHandle, &tx_frame, 0, osWaitForever); 
    }
}


void StartMotorTask(void *argument)
{
    const TickType_t xDelay = pdMS_TO_TICKS(100); 
    TickType_t xLastWakeTime;
    uint32_t ulNotificationValue;

    xLastWakeTime = xTaskGetTickCount();
    MotorTaskHandle = xTaskGetCurrentTaskHandle(); // 设置句柄

    /* Infinite loop */
    for(;;)
    {
        // 阻塞等待：可以被周期定时唤醒 (xDelay) 或被 Task Notification 唤醒
        
        // xTaskNotifyWait 默认是阻塞的，超时时间为 xDelay
        if (xTaskNotifyWait(0,                      // ulBitsToClearOnEntry: 不清除
                            0,                      // ulBitsToClearOnExit: 不清除
                            &ulNotificationValue,   // pulNotificationValue: 接收通知值
                            xDelay) == pdPASS) 
        {
            // **被通知唤醒 (如 CMD_ID_REQUEST_ALL_STATUS)**
            if (ulNotificationValue == CMD_ID_REQUEST_ALL_STATUS) {
                // 立即执行采集和发送
                MotorTask_SendReport();
            }
        }
        
        // 周期时间到或被通知后，检查是否需要执行周期性发送
        if (xTaskGetTickCount() - xLastWakeTime >= xDelay) {
            MotorTask_SendReport();
            xLastWakeTime = xTaskGetTickCount();
        }
    }
}