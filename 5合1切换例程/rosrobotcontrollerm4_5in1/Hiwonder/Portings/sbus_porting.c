/**
 * @file lwmem_porting.c
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief SBUS 协议接收的硬件相关处理
 * @version 0.1
 * @date 2023-05-18
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <stdio.h>
#include "SBus.h"
#include "lwmem_porting.h"
#include "lwrb.h"
#include "usart.h"
#include "global.h"
#include "gui_guider.h"


#define SBUS_RX_DMA_BUFFER_SIZE 32 /* SBUS DMA 接收缓存长度 */
#define SBUS_RX_FIFO_BUFFER_SIZE 512 /* SBUS FIFO 缓存长度 */

/* SBUS1 相关变量 */
static uint8_t *subs1_rx_dma_buffers[2];
static uint32_t sbus1_rx_dma_buffer_index;
static uint8_t *sbus1_rx_fifo_buffer;
static lwrb_t *sbus1_rx_fifo;
static void sbus_dma_receive_event_callback(UART_HandleTypeDef *huart, uint16_t length);

/* SBUS 对外暴露变量 */
SBusStatusObjectTypeDef *sbus1_status;


/* SBUS1 需要的外部变量 */
extern osSemaphoreId_t sbus_data_readyHandle; /* subs数据就绪信号量Handle */
extern osSemaphoreId_t sbus_data_ready_01_Handle;
extern osEventFlagsId_t sbus_data_ready_event_Handle;


/**
 * @brief  SBUS初始化
 *  初始化 SBUS 相关内存、变量并开始SBUS 接收
 * @retval None.
 */
void sbus_init(void)
{
    sbus1_status = LWMEM_CCM_MALLOC(sizeof(SBusStatusObjectTypeDef));
    sbus1_status->type_id = OBJECT_TYPE_ID_GAMEPAD_STATUS;
    /* DMA 接收缓存初始化 */
    subs1_rx_dma_buffers[0] = LWMEM_RAM_MALLOC(SBUS_RX_DMA_BUFFER_SIZE); /* DMA 缓存不能放在 CCMRAM 上 */
    subs1_rx_dma_buffers[1] = LWMEM_RAM_MALLOC(SBUS_RX_DMA_BUFFER_SIZE);
    sbus1_rx_dma_buffer_index = 0;

    /* 接收 FIFO 初始化 */
    sbus1_rx_fifo_buffer  = LWMEM_CCM_MALLOC(SBUS_RX_FIFO_BUFFER_SIZE);
    sbus1_rx_fifo = LWMEM_CCM_MALLOC(sizeof(lwrb_t));
    lwrb_init(sbus1_rx_fifo, sbus1_rx_fifo_buffer, SBUS_RX_FIFO_BUFFER_SIZE);

}


/**
 * @brief  SBUS接收事件回调
 *  在 DMA 接收缓存满后或者空闲时触发本函数将接收到的数据压入接收FIFO缓存，再由接收任务完成解析和处理
 *  因为SBUS协议的发送最小间隔为4ms, 而DMA缓存大小设为了32, 所以总是触发空闲事件而不触发DMA满事件
 *  @attention 需要注意, 不是所有遥控接收器都会一次性完成一帧的发送，所以一帧数据可能会触发多次空闲事件, 不能直接使用空闲事件进行帧分割。
 *             因为接收开始位置不一定是帧头位置，所以也不能直接使用DMA接收长度进行分割。
 *             如果一定要使用DMA长度/空闲事件进行帧分割就必须先完成一次帧同步，即接收到一个完整、正确的帧再开始DMA接收确保DMA接收的首字节为帧头。
 *             在本实现中，将所有接收到的数据汇聚为字节流，再通过帧头、帧尾的固定值进行分割。
 * @param huart 串口实例
 * @param Pos 接收到的字节数
 * @retval None.
 */
static void sbus_dma_receive_event_callback(UART_HandleTypeDef *huart, uint16_t length)
{
    uint32_t cur_index = sbus1_rx_dma_buffer_index; /* 取得当前DMA缓存的索引号 */
    sbus1_rx_dma_buffer_index ^= 1;
    if(length < SBUS_RX_DMA_BUFFER_SIZE) {
		//终止接收
        HAL_UART_AbortReceive(&huart5);
    }
	//以DMA方式接收数据
    HAL_UARTEx_ReceiveToIdle_DMA(	&huart5, 
									(uint8_t*)subs1_rx_dma_buffers[sbus1_rx_dma_buffer_index], 
									SBUS_RX_DMA_BUFFER_SIZE);
    lwrb_write(sbus1_rx_fifo, subs1_rx_dma_buffers[cur_index], length); /* 将接收到的数据写入fifo ring */
    if(lwrb_get_full(sbus1_rx_fifo) >= 25) { /* SBUS 每帧数据大小为25个字节， 每一次超过25个字节就进行一次解析 */
        osSemaphoreRelease(sbus_data_ready_01_Handle); /* 置位 SBUS 数据就绪信号量 */
    }
}

#define  MAX_SBUS_DATA  1400
#define  MIN_SBUS_DATA  600


static void deal_SbusData( SBusStatusObjectTypeDef *status );
static char A_T_C(int16_t analog_x, int16_t analog_y);

/**
 * @brief  SBUS接收任务入口函数
 * @param argument 入口参数
 * @retval None.
 */
#if ENABLE_SBUS
void sbus_rx_task_entry(void *argument)
{
	//SBUS接收缓冲区
    static uint8_t buf_temp[32];
	
	//初始化SBUS
	sbus_init();
    
	/* 开始接收 */
//跳转标签
start_sbus_receive:
	//中止串口接收数据，防止数据
    HAL_UART_AbortReceive(&huart5);
    HAL_UART_RegisterRxEventCallback(&huart5, sbus_dma_receive_event_callback); /* 注册接收事件回调 */
    /* 使用 ReceiveToIdle_DMA 进行接收， 该函数会在DMA缓存满时中断或在接收空闲时中断并触发接收事件回调 */
    HAL_UARTEx_ReceiveToIdle_DMA(	&huart5, 
									(uint8_t*)subs1_rx_dma_buffers[sbus1_rx_dma_buffer_index], 
									SBUS_RX_DMA_BUFFER_SIZE);
    /* 解析循环 */
    for(;;) {
        if(osSemaphoreAcquire(sbus_data_ready_01_Handle, 1000) != osOK) { /* 等待数据就绪 */
            goto start_sbus_receive;  //跳转至 start_sbus_receive 标签
        }
        for(;;) {
            memset(buf_temp, 0, 25); //清空缓冲区
            if(lwrb_peek(sbus1_rx_fifo, 0, buf_temp, 25) == 25) {  /* 从缓存中读25个字节 */
                if(sbus_decode_frame(buf_temp, sbus1_status) == 0) { /* 尝试解析直到字节不够 */
                    lwrb_skip(sbus1_rx_fifo, 25);  //丢弃已读取数据
					if(sbus1_status->signal_loss == false)	//若已连接
					{
//						sbus_print_status(sbus1_status);	//打印遥控器的各个数据
						deal_SbusData(sbus1_status);	//数据处理函数
					}
                } else {  //若解析不了，则跳到下一个字节，继续解析
                    lwrb_skip(sbus1_rx_fifo, 1);
                }
            } else {
                break;
            }
        }
    }
}

/* 航模数据处理函数 */
static void deal_SbusData( SBusStatusObjectTypeDef *status )
{
	//外部声明运动控制队列句柄
	extern osMessageQueueId_t moving_ctrl_queueHandle;
	
	{//左摇杆
		static char direction_msg_l = 'I';
		static char last_direction_msg_l = 'I';
		//对左摇杆的位置做映射
		direction_msg_l = A_T_C(status->channels[3], status->channels[2]);
		if(direction_msg_l != 'I') { //若不是停止信号
			//判断 拨钮二 所选的模式  平移或自旋，选择信号
			if( direction_msg_l == 'C')	
			{
				//摇杆回中的值为992，范围为[992-800 ， 992+800]，
				//若拨钮有两档，则只有192和1792两个值，三档则为192、992、1792。
				direction_msg_l = status->channels[5] > 992 ? direction_msg_l : 'l';
			}
			if( direction_msg_l == 'G')
			{
				direction_msg_l = status->channels[5] > 992 ? direction_msg_l : 'p';
			}
			osMessageQueuePut(moving_ctrl_queueHandle, &direction_msg_l, 0, 0); //向队列发送控制信号
		} else {
			if(last_direction_msg_l != 'I') { /* 只在松开时发送一次松开信号 */
				osMessageQueuePut(moving_ctrl_queueHandle, &direction_msg_l, 0, 0);
			}
		}
		//记录该值
		last_direction_msg_l = direction_msg_l;
	}
	
	
	static char direction_msg_r = 'I';
	static char last_direction_msg_r = 'I';
	if( status->channels[6] < 992 )	//锁定回中  拨钮三控制云台回中锁定、解锁
	{
		if( 'f' != last_direction_msg_r )
		{
			//发送一次回中指令，不让摇杆控制云台旋转，则达到锁定效果
			direction_msg_r = 'f';
			last_direction_msg_r = direction_msg_r;
			osMessageQueuePut(moving_ctrl_queueHandle, &direction_msg_r, 0, 0);
		}
	}else{	//若云台不锁定
		{//右摇杆
			static uint8_t send_count = 0;
			//进行摇杆的位置信号转换
			direction_msg_r = A_T_C(status->channels[0], status->channels[1]);
			if(direction_msg_r != 'I') {
				send_count++;
				//减缓发送信息的速度 尽量控制在每秒发送20次信号以内的速度，能达到好的舵机控制效果
				if( 4 == send_count )	
				{
					send_count = 0;
					last_direction_msg_r = direction_msg_r;
					//将信号映射到 J 至 R 
					direction_msg_r = direction_msg_r - 'A' + 'J';
					//发送控制信号
					osMessageQueuePut(moving_ctrl_queueHandle, &direction_msg_r, 0, 0);
				}
			} else {
				if(last_direction_msg_r != 'I') { /* 只在松开时发送一次松开信号 */
					send_count++;
					//这里是由于信号发送的频率太快，计数4次再发送
					if( 4 == send_count )	
					{
						send_count = 0;
						last_direction_msg_r = direction_msg_r;
						direction_msg_r = direction_msg_r - 'A' + 'J';
						osMessageQueuePut(moving_ctrl_queueHandle, &direction_msg_r, 0, 0);
					}
				}
			}
		}
	}
	
	static char msg_l = 'R';
	
	static int16_t last_data_1 = 992;
	if( status->channels[4] > 1400 ) //拨钮一 当前为往下打  
	{
		if(last_data_1 < 1400 )	//上一次不是往下打，则蜂鸣器响一声  
		{
			msg_l = 'S';	//
			osMessageQueuePut(moving_ctrl_queueHandle, &msg_l, 0, 0);
		}
		last_data_1 = status->channels[4]; //记忆这次拨钮值  
	}else if( status->channels[4] < 600 ) //拨钮一 当前为往上打  
	{	//则蜂鸣器间歇性鸣叫   
		static uint8_t send_count = 0;
		send_count++;
		if( 100 == send_count )	
		{
			send_count = 0;
			msg_l = 'S';	//
			osMessageQueuePut(moving_ctrl_queueHandle, &msg_l, 0, 0);
		}
		last_data_1 = status->channels[4]; //记忆这次拨钮值
	}else{ //若为中间，则无动作
		last_data_1 = status->channels[4]; //记忆这次拨钮值
	}

	static int16_t last_data_4 = 992;
	if( status->channels[7] > 1600 ) //拨钮四 当前为往下打
	{
		if(last_data_4 < 1600 )	//上一次不是往下打，则发送一次减速信号
		{
			msg_l = 'n';	//减速
			osMessageQueuePut(moving_ctrl_queueHandle, &msg_l, 0, 0);
		}
		last_data_4 = status->channels[7]; //记忆这次拨钮值
	}else if( status->channels[7] < 300 ) //拨钮四 当前为往上打
	{
		if(last_data_4 > 300 )	//上一次不是往上打，则发送一次加速信号
		{
			msg_l = 'j';	//加速
			osMessageQueuePut(moving_ctrl_queueHandle, &msg_l, 0, 0);
		}
		last_data_4 = status->channels[7]; //记忆这次拨钮值
	}else{ //若为中间，则无动作
		static uint8_t count = 0;
		count++;
		if( 20 < count )  //防抖
		{
			count = 0;
			last_data_4 = status->channels[7]; //记忆这次拨钮值
		}
	}
	
	
}

/* 将摇杆对应区域映射为相对应的字符 */
static char A_T_C(int16_t analog_x, int16_t analog_y)
{
    char result = ' ';
    if(analog_x < MIN_SBUS_DATA) {
        if(analog_y < MIN_SBUS_DATA) {
            result = 'D';
        } else if(analog_y > MAX_SBUS_DATA) {
            result = 'B';
        } else {
            result = 'C';
        }
    } else if(analog_x > MAX_SBUS_DATA) {
        if(analog_y < MIN_SBUS_DATA) {
            result = 'F';
        } else if(analog_y > MAX_SBUS_DATA) {
            result = 'H';
        } else {
            result = 'G';
        }
    } else {
        if(analog_y < MIN_SBUS_DATA) {
            result = 'E';
        } else if(analog_y > MAX_SBUS_DATA) {
            result = 'A';
        } else {
            result = 'I';
        }
    }
    return result;
}

#endif

