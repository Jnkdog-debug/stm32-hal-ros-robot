#include "uart_tasks.h" // 包含任务函数声明
#include "protocol.h"
#include "uart_driver.h"
#include "cmsis_os.h" // 确保包含 RTOS API

// 引用 CubeMX 自动生成的句柄 (CMSIS API)
extern osMessageQueueId_t UartTxQueueHandle;
extern osSemaphoreId_t UartTxDoneSemaphoreHandle;

// 引用在 freertos.c 中定义的任务句柄（供 ISR 使用）
extern TaskHandle_t UartRxTaskHandle;

// 定义在 uart_driver.h 中声明的全局句柄（供 IDLE ISR 使用）
TaskHandle_t UartRxTaskHandle_ISR = NULL;

// 定义 FreeRTOS 原生队列句柄，将 CMSIS RTOS 的 osMessageQueueId_t 转换为 QueueHandle_t
// osMessageQueueId_t 实际上是 QueueHandle_t 的包装
QueueHandle_t UartTxQueue = NULL; 


/**
 * @brief Function implementing the UartRxTask thread.
 */
void StartUartRxTask(void *argument)
{
    // 初始化 FreeRTOS 原生队列句柄（将 CMSIS RTOS 句柄转换为 FreeRTOS 句柄）
    // osMessageQueueId_t 在 CMSIS RTOS 中实际上就是 QueueHandle_t
    UartTxQueue = (QueueHandle_t)UartTxQueueHandle;
    
    // 任务私有缓冲区：用于拷贝接收到的数据帧
    // 注意：这个缓冲区大小需要至少能容纳一帧最大的数据包。
    // 我们使用和 DMA 环形缓冲区一样的大小，简化处理。
    uint8_t local_rx_buffer[UART_RX_BUFFER_SIZE]; 
    uint32_t received_length = 0;
    
    // 1. 设置任务句柄 (供 IDLE 中断通知使用)
    // 必须在任务内部调用，因为任务句柄只有在任务运行时才确定。
    UartRxTaskHandle = xTaskGetCurrentTaskHandle();
    
    // 2. 启动 DMA 循环接收
    // 必须在 IDLE 中断启用之前或同时启动
    UART_Start_DMA_Receive(&huart3); 
    
    /* Infinite loop */
    for(;;)
    {
        // 3. 阻塞等待 IDLE 中断通知
        // xTaskNotifyWait：等待来自 ISR 的通知，并接收通知值 (即数据长度)
        // 参数：ulClearOnEntry=0 (不清除), ulClearOnExit=0 (不清除)，但 eSetValueWithOverwrite 模式下，ulValue=received_length
        if (xTaskNotifyWait(0, 0, &received_length, portMAX_DELAY) == pdPASS) {
            
            // 4. 检查数据有效性
            if (received_length == 0 || received_length > UART_RX_BUFFER_SIZE) {
                // 收到空通知或长度异常，忽略
                continue;
            }

            // 5. 数据拷贝 (核心：将 DMA 缓冲区的数据拷贝到任务私有栈)
            // 拷贝的原因：DMA 已经重新启动，正在写入环形缓冲区，防止数据被覆盖。
            // 必须使用 DMA 缓冲区的前 received_length 字节。
            memcpy(local_rx_buffer, uart_rx_dma_buffer, received_length);

            // 6. 协议解析
            protocol_frame_t rx_frame;
            protocol_status_t status = Protocol_ParseFrame(local_rx_buffer, received_length, &rx_frame);
            
            // 7. 结果处理
            if (status == PROTOCOL_OK) {
                
                // 协议解析成功，进行命令分发
                Protocol_DispatchCommand(&rx_frame);
                
            } else {
                // 协议解析失败（校验错误、帧头错误等）
                // 可以选择打印错误或发送一个错误应答帧 (NACK)
                // 例如：Protocol_SendNACK(rx_frame.cmd_id, status);
            }
        }
    }
}



/**
 * @brief Function implementing the UartTxTask thread.
 */
void StartUartTxTask(void *argument)
{
    protocol_frame_t tx_frame;
    
    // 将 CMSIS 句柄转换为 FreeRTOS Native 句柄，方便同步操作
    SemaphoreHandle_t tx_done_semaphore_native = UartTxDoneSemaphoreHandle;
    
    /* Infinite loop */
    for(;;)
    {
        // 1. 阻塞等待发送队列
        // osMessageQueueGet: 阻塞等待 UartTxQueueHandle 中有数据帧
        if (osMessageQueueGet(UartTxQueueHandle, &tx_frame, NULL, osWaitForever) == osOK) {
            
            // 2. 计算总长度
            // 长度 = 帧头 + 载荷长度 + CRC长度
            uint16_t total_len = FRAME_HEADER_SIZE + tx_frame.payload_len + FRAME_CRC_SIZE;
            
            // 3. 启动 DMA 发送
            // 注意：我们必须使用 non-blocking 的 DMA 发送函数
            HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart3, (uint8_t*)&tx_frame, total_len);
            
            if (status == HAL_OK) {
                
                // 4. 等待发送完成信号量
                // 在 HAL_UART_TxCpltCallback 触发之前，任务将在这里阻塞
                // 使用 FreeRTOS Native API 来等待信号量
                xSemaphoreTake(tx_done_semaphore_native, portMAX_DELAY);
                
                // 信号量被释放后，发送完成，任务继续循环，处理队列中的下一个数据包。
                
            } else {
                // DMA 发送启动失败 (例如，DMA 忙碌)
                // 此时可以记录错误，或进行短时间延时后重试 (这里选择直接继续，丢弃当前包)
            }
        }
        
        // 任务循环非常快，不需要 osDelay，因为它主要被 osMessageQueueGet 阻塞
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    // 检查是否是目标 UART
    if (huart->Instance == huart3.Instance) { 
        // 将 CMSIS 信号量句柄转换为 FreeRTOS Native 句柄
        SemaphoreHandle_t tx_done_semaphore_native = UartTxDoneSemaphoreHandle;
        
        if (tx_done_semaphore_native != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            // 释放信号量，唤醒 Tx Task
            xSemaphoreGiveFromISR(tx_done_semaphore_native, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}


