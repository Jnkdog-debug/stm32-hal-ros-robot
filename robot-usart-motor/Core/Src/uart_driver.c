#include "uart_driver.h"
#include "usart.h" // 包含 CubeMX 生成的 huart 句柄

// 定义 DMA 缓冲区
uint8_t uart_rx_dma_buffer[UART_RX_BUFFER_SIZE];

extern TaskHandle_t UartRxTaskHandle; // 声明外部任务句柄
/**
 * @brief  启动 UART 的 DMA 循环接收
 * @param  huart: 指向 UART 句柄的指针
 */
void UART_Start_DMA_Receive(UART_HandleTypeDef* huart) {
    // 确保 huart3 是您的目标 UART 句柄
    HAL_UART_Receive_DMA(huart, uart_rx_dma_buffer, UART_RX_BUFFER_SIZE);
}

/**
 * @brief  IDLE 中断处理逻辑
 * @param  huart: 指向 UART 句柄的指针
 * @note   该函数应在 UART 中断服务程序中被调用。
 */
void UART_IDLE_Process(UART_HandleTypeDef *huart) {
    // 确保是 IDLE 标志触发
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET) {
        
        // 1. 清除 IDLE 标志 (读 SR 寄存器，写 DR 寄存器或直接清除)
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        
        // 2. 停止 DMA 传输
        HAL_UART_DMAStop(huart);
        
        // 3. 计算接收到的数据长度
        // NDC: Number of Data remaining in the DMA buffer
        uint16_t current_dma_count = __HAL_DMA_GET_COUNTER(huart->hdmarx);
        uint16_t received_length = UART_RX_BUFFER_SIZE - current_dma_count;

        // 4. 通知接收任务
        if (UartRxTaskHandle != NULL && received_length > 0) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            // 使用 Task Notification 传递长度信息
            xTaskNotifyFromISR(UartRxTaskHandle, received_length, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

        // 5. 重新启动 DMA 循环接收，为下一帧做准备
        UART_Start_DMA_Receive(huart);
    }
}