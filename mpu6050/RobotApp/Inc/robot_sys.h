#ifndef ROBOT_SYS_H
#define ROBOT_SYS_H

#include "main.h"
#include "cmsis_os.h" // 包含 FreeRTOS 定义
#include "usart.h"
#include "mpu6050.h"


//消息类型定义
/* 1. 定义消息类型枚举 */
typedef enum {
    MSG_TYPE_IMU,       // IMU 数据
    MSG_TYPE_MOTOR,     // 电机速度数据
    MSG_TYPE_BATTERY    // 电池数据
} MsgType_t;

/* 2. 定义通用消息信封 (这就是放入队列的东西) */
typedef struct {
    MsgType_t type;     // 标签：里面装的啥？
    void* p_data;       // 内容：指向数据的指针 (void* 可以指向任何类型)
} TxMessage_t;

/* 3. 定义电机数据结构体 (举例) */
typedef struct {
// V2.0 麦轮状态上报结构体 (总计 36 字节)
    float   rpm[4];             // 4个轮子的转速 (16字节)
    int32_t count[4];           // 4个轮子的累计脉冲 (16字节)
    float   battery_voltage;    // 电池电压 (4字节)
 } MotorData_t;




//机器人全局变量定义
// 1. 定义全局结构体类型
typedef struct {
    uint8_t is_running;
    uint32_t sample_period;  // 采样周期 (ms)
    
// 【新增】V2.0 麦轮全局空间速度指令
    float target_Vx;
    float target_Vy;
    float target_Wz;
    uint32_t last_cmd_time; // 喂狗时间戳

     float battery_voltage;  // 电池电压 (V)

    

} Robot_State_t;



// 2. 声明全局变量 (extern)
// 告诉所有引用这个文件的 c 文件：去别的地方找这些变量，别自己重新创建
extern Robot_State_t g_robot;
extern osMutexId_t robotMutexHandle;
extern uint8_t rx_buffer_byte;

// 3. 声明队列句柄
extern osMessageQueueId_t cmdQueueHandle;
extern osMessageQueueId_t dataQueueHandle;

#endif