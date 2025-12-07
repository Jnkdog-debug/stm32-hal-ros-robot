#include "protocol.h"
#include "uart_tasks.h" // 需要访问发送队列
#include <string.h>     // 用于 memcpy

// -------------------------------------------------------------
// CRC16-CCITT (0x1021) 实现
// -------------------------------------------------------------

// CRC 表定义（全局静态变量，只在文件内部使用）
static uint16_t const crc16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDE, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

/**
 * @brief  计算 CRC16 校验和 (CRC-16/CCITT-FALSE, Poly: 0x1021, Init: 0xFFFF)
 * @param  data: 待计算数据指针
 * @param  len: 数据长度
 * @retval 16位 CRC 校验和
 */
uint16_t Protocol_CalculateCRC16(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF; // 初始值 0xFFFF

    for (uint16_t i = 0; i < len; i++) {
        // 1. 取当前 CRC 的高 8 位作为查表索引，并与当前数据字节异或
        uint8_t index = (uint8_t)(crc >> 8) ^ data[i];
        
        // 2. 将当前 CRC 左移 8 位
        crc = crc << 8;

        // 3. 将 CRC 值与查表结果异或

        crc ^= crc16_table[index];
    }
    
    return crc;
}
// -------------------------------------------------------------
// CRC 函数结束
// -------------------------------------------------------------
// --------------------------------------------------------------------------

/**
 * @brief  打包数据帧（用于发送）
 */
protocol_status_t Protocol_PackFrame(protocol_frame_t *frame, uint8_t cmd_id, const uint8_t *payload, uint8_t len) {
    if (len > MAX_PAYLOAD_SIZE) return PROTOCOL_ERR_LENGTH;

    frame->header[0] = FRAME_HEADER_0;
    frame->header[1] = FRAME_HEADER_1;
    frame->cmd_id = cmd_id;
    frame->payload_len = len;

    if (payload != NULL && len > 0) {
        memcpy(frame->payload, payload, len);
    }
    
    // 计算校验和：从帧头开始到 payload 结束
    uint16_t crc_len = FRAME_HEADER_SIZE + len;
    frame->checksum = Protocol_CalculateCRC16((uint8_t*)frame, crc_len);

    return PROTOCOL_OK;
}

/**
 * @brief  解析接收到的原始数据（用于接收）
 */
protocol_status_t Protocol_ParseFrame(const uint8_t *raw_data, uint16_t raw_len, protocol_frame_t *out_frame) {
    // 1. 最小长度和帧头检查
    uint16_t min_len = FRAME_HEADER_SIZE + FRAME_CRC_SIZE;
    if (raw_len < min_len) return PROTOCOL_ERR_LENGTH;
    if (raw_data[0] != FRAME_HEADER_0 || raw_data[1] != FRAME_HEADER_1) return PROTOCOL_ERR_HEADER;

    // 2. 载荷长度检查
    uint8_t payload_len = raw_data[3];
    uint16_t expected_total_len = FRAME_HEADER_SIZE + payload_len + FRAME_CRC_SIZE;
    if (raw_len != expected_total_len || payload_len > MAX_PAYLOAD_SIZE) return PROTOCOL_ERR_LENGTH;

    // 3. CRC 校验
    uint16_t crc_data_len = FRAME_HEADER_SIZE + payload_len; // 校验范围：从帧头到载荷结束
    uint16_t calculated_crc = Protocol_CalculateCRC16(raw_data, crc_data_len);
    
    // 提取帧中的校验和 (它位于 payload 之后)
    uint16_t received_crc = *(uint16_t*)(raw_data + crc_data_len);
    
    if (calculated_crc != received_crc) return PROTOCOL_ERR_CHECKSUM;

    // 4. 解析成功，填充 out_frame (这里需要将原始数据拷贝到结构体)
    memcpy(out_frame, raw_data, expected_total_len);

    return PROTOCOL_OK;
}

/**
 * @brief  发送通用应答 (ACK)
 */
void Protocol_SendACK(uint8_t ack_cmd_id) {
    protocol_frame_t ack_frame;
    // ACK 的 payload 为原始命令 ID
    Protocol_PackFrame(&ack_frame, CMD_ID_ACK, &ack_cmd_id, 1); 
    
    // 将 ACK 帧投入发送队列
    // 注意：这里的 UartTxQueue 必须先在 uart_tasks.c 中定义
    if (UartTxQueue != NULL) {
        xQueueSend(UartTxQueue, &ack_frame, 0); // 0 等待时间，非阻塞投入
    }
}

extern TaskHandle_t MotorTaskHandle;

// 假设有一个配置结构体 (全局变量，需要互斥锁保护，但此处简化)
typedef struct {
    uint16_t pid_kp;
    uint16_t pid_ki;
} SystemConfig_t;

static SystemConfig_t system_config = { .pid_kp = 100, .pid_ki = 10 };

/**
 * @brief  命令分发和执行
 */
void Protocol_DispatchCommand(const protocol_frame_t *rx_frame) {
    uint8_t cmd_id = rx_frame->cmd_id;

    switch (cmd_id) {
        case CMD_ID_REQUEST_ALL_STATUS:
            // 收到请求所有状态的命令 (0x10)
            
            // 1. 通知 MotorTask 立即发送数据 (使用通知值传递命令 ID)
            if (MotorTaskHandle != NULL) {
                xTaskNotify(MotorTaskHandle, CMD_ID_REQUEST_ALL_STATUS, eSetValueWithOverwrite);
            }
            
            // 2. 发送 ACK 确认命令已收到
            Protocol_SendACK(cmd_id);
            break;
            
        case CMD_ID_SET_PARAM:
            // 收到设置系统参数的命令 (0x20)
            
            // 1. 校验长度：确保载荷足够大
            if (rx_frame->payload_len == sizeof(SystemConfig_t)) {
                
                // 2. 更新配置
                // 注意：在多任务环境下，修改全局变量应使用 Mutex 互斥锁保护！此处为简化而省略。
                memcpy(&system_config, rx_frame->payload, sizeof(SystemConfig_t));
                
                // 3. 发送 ACK 确认设置成功
                Protocol_SendACK(cmd_id);
                
            } else {
                // 载荷长度错误，发送 NACK (如果实现了 NACK)
                // Protocol_SendNACK(cmd_id, PROTOCOL_ERR_LENGTH);
            }
            break;

        default:
            // 收到未知命令，忽略或发送 NACK
            break;
    }
}