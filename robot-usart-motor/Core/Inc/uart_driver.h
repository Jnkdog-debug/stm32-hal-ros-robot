#ifndef __UART_DRIVER_H
#define __UART_DRIVER_H

#include "main.h"
#include "cmsis_os.h" // 包含 FreeRTOS API
#include <string.h>

// 宏定义：DMA 环形缓冲区大小
#define UART_RX_BUFFER_SIZE 512

// 全局变量：用于 DMA 接收的缓冲区
extern uint8_t uart_rx_dma_buffer[UART_RX_BUFFER_SIZE];

// 全局变量：用于被 IDLE 中断通知的 Task 句柄
extern TaskHandle_t UartRxTaskHandle;

/* 函数声明 */

// 启动 DMA 接收的初始化函数
void UART_Start_DMA_Receive(UART_HandleTypeDef* huart);

// IDLE 中断处理函数 (供 usart.c 调用)
void UART_IDLE_Process(UART_HandleTypeDef *huart);

#endif /* __UART_DRIVER_H */