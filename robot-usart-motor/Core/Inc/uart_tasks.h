#ifndef __UART_TASKS_H
#define __UART_TASKS_H

#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "protocol.h"
#include "usart.h" // 包含 huart 句柄

// FreeRTOS 队列声明
#define UART_TX_QUEUE_LENGTH 10
extern QueueHandle_t UartTxQueue;

// 发送完成信号量 (用于 DMA 非阻塞管理)
extern SemaphoreHandle_t UartTxDoneSemaphore;


/* --- 任务函数声明 --- */
void StartUartRxTask(void *argument);
void StartUartTxTask(void *argument);
void StartMPU6050Task(void *argument);

#endif /* __UART_TASKS_H */