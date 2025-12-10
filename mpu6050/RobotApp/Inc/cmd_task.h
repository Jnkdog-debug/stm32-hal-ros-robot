#ifndef CMD_TASK_H
#define CMD_TASK_H


#define MAX_PAYLOAD_LEN 64

/* 定义新的状态 */
typedef enum {
    STATE_WAIT_HEADER_1,
    STATE_WAIT_HEADER_2,
    STATE_WAIT_FUNC,
    STATE_WAIT_LEN,
    STATE_WAIT_PAYLOAD,
    STATE_WAIT_CRC
} FrameState_t;



// 声明任务的主体逻辑函数
void StartcmdTask(void *argument);

#endif