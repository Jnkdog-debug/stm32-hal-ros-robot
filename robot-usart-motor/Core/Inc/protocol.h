#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "main.h"

// --- 协议常量定义 ---
#define FRAME_HEADER_0  0xAA  // 帧头第一个字节
#define FRAME_HEADER_1  0x55  // 帧头第二个字节
#define MAX_PAYLOAD_SIZE 64   // 帧载荷最大长度
#define FRAME_HEADER_SIZE 4   // 帧头 (AA 55 ID LEN)
#define FRAME_CRC_SIZE    2   // CRC16 占 2 字节

// --- 协议状态码 ---
typedef enum {
    PROTOCOL_OK = 0,
    PROTOCOL_ERR_CHECKSUM,
    PROTOCOL_ERR_LENGTH,
    PROTOCOL_ERR_HEADER,
    PROTOCOL_ERR_UNKNOWN_CMD,
} protocol_status_t;

// --- 命令/数据 ID 定义 ---
#define CMD_ID_ENCODER_REPORT       0x01 // 编码器数据上报 (TX)
#define CMD_ID_SENSOR_REPORT        0x02 // 传感器数据上报 (TX)
#define CMD_ID_REQUEST_ALL_STATUS   0x10 // 请求所有状态 (RX)
#define CMD_ID_SET_PARAM            0x20 // 设置系统参数 (RX)
#define CMD_ID_ACK                  0xF0 // 通用应答 (TX/RX)


// --- 数据帧结构体定义 ---
typedef struct {
    uint8_t  header[2];    // 0xAA 0x55
    uint8_t  cmd_id;       // 命令或数据类型 ID 
    uint8_t  payload_len;  // 载荷长度 (0 - MAX_PAYLOAD_SIZE)
    uint8_t  payload[MAX_PAYLOAD_SIZE]; // 实际数据
    uint16_t checksum;     // 校验和 (CRC16)
} __attribute__((packed)) protocol_frame_t;


/* --- 函数声明 --- */

// 计算 CRC16 校验和
uint16_t Protocol_CalculateCRC16(const uint8_t *data, uint16_t len);

// 打包数据帧（用于发送）
protocol_status_t Protocol_PackFrame(protocol_frame_t *frame, uint8_t cmd_id, const uint8_t *payload, uint8_t len);

// 解析接收到的原始数据（用于接收）
protocol_status_t Protocol_ParseFrame(const uint8_t *raw_data, uint16_t raw_len, protocol_frame_t *out_frame);

// 命令分发和执行
void Protocol_DispatchCommand(const protocol_frame_t *rx_frame);

// 辅助函数：发送通用应答
void Protocol_SendACK(uint8_t ack_cmd_id);

#endif /* __PROTOCOL_H */