#!/usr/bin/env python3
"""接收 STM32 自动发送的编码器数据"""

import serial
import struct
import time

def crc16_ccitt(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            crc <<= 1
            if crc & 0x10000:
                crc ^= 0x1021
            crc &= 0xFFFF
    return crc

def parse_frame(data):
    """解析协议帧"""
    if len(data) < 8:
        return None
    
    # 检查帧头
    if data[0] != 0xAA or data[1] != 0x55:
        return None
    
    cmd_id = data[2]
    payload_len = data[3]
    
    # 检查数据长度
    if len(data) < 4 + payload_len + 2:
        return None
    
    # 提取 payload 和 CRC
    payload = data[4:4+payload_len]
    crc_recv = struct.unpack('<H', data[4+payload_len:6+payload_len])[0]
    
    # 验证 CRC
    crc_calc = crc16_ccitt(data[:4+payload_len])
    if crc_recv != crc_calc:
        print(f"❌ CRC 错误: 接收={crc_recv:04X}, 计算={crc_calc:04X}")
        return None
    
    return {
        'cmd_id': cmd_id,
        'payload_len': payload_len,
        'payload': payload,
        'crc': crc_recv
    }

def receive_data(port, baudrate=115200):
    """从串口接收数据"""
    ser = serial.Serial(port, baudrate, timeout=1)
    time.sleep(0.5)
    
    print(f"[连接到 {port} @ {baudrate}bps]")
    print("等待数据...\n")
    
    buffer = bytearray()
    frame_count = 0
    
    try:
        while True:
            if ser.in_waiting:
                # 读取一个字节
                byte = ser.read(1)
                buffer.extend(byte)
                
                # 查找帧头
                if len(buffer) >= 2 and buffer[0] == 0xAA and buffer[1] == 0x55:
                    # 获取 payload 长度
                    if len(buffer) >= 4:
                        payload_len = buffer[3]
                        frame_size = 4 + payload_len + 2  # header + cmd + len + payload + crc
                        
                        # 等待完整帧
                        if len(buffer) >= frame_size:
                            frame_data = bytes(buffer[:frame_size])
                            
                            # 解析帧
                            frame = parse_frame(frame_data)
                            if frame:
                                frame_count += 1
                                
                                # 显示帧信息
                                print(f"[帧 #{frame_count}]")
                                print(f"  原始数据: {frame_data.hex().upper()}")
                                print(f"  命令 ID: 0x{frame['cmd_id']:02X}", end="")
                                
                                if frame['cmd_id'] == 0x01:
                                    print(" (ENCODER_REPORT)")
                                    # 解析编码器计数
                                    if frame['payload_len'] == 4:
                                        count = struct.unpack('<I', frame['payload'])[0]
                                        print(f"  编码器计数: {count}")
                                elif frame['cmd_id'] == 0x02:
                                    print(" (SENSOR_REPORT)")
                                elif frame['cmd_id'] == 0xF0:
                                    print(" (ACK)")
                                else:
                                    print(f" (未知)")
                                    print(f"  Payload: {frame['payload'].hex().upper()}")
                                
                                print()
                            
                            # 删除已处理的数据
                            buffer = buffer[frame_size:]
                elif len(buffer) > 100:
                    # 缓冲区过大，清空
                    buffer = bytearray()
            
            time.sleep(0.01)
    
    except KeyboardInterrupt:
        print("\n[断开连接]")
    finally:
        ser.close()

if __name__ == '__main__':
    receive_data('/dev/ttyACM0')  # 修改为实际串口