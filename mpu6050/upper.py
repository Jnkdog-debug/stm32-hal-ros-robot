import serial
import struct
import threading
import time
import sys

# ================= 配置区 =================
# Windows用 'COMx', Linux用 '/dev/ttyUSB0'
SERIAL_PORT = '/dev/ttyACM0'  
BAUD_RATE = 115200

# ================= 协议定义 =================
# 帧头
HEADER = b'\xAA\x55'

# 功能码
FUNC_IMU   = 0x10
FUNC_MOTOR = 0x20
CMD_START  = 0x01
CMD_STOP   = 0x02
CMD_SPEED  = 0x10

# 全局状态
running = True
target_vx = 0.0 # 线速度
target_vw = 0.0 # 角速度

# ================= CRC 校验 =================
def calc_checksum(data_bytes):
    return sum(data_bytes) & 0xFF

# ================= 发送线程 =================
def send_thread(ser):
    global target_vx, target_vw
    last_vx = 0.0
    last_vw = 0.0
    
    print("[TX] Sending thread started...")
    
    while running:
        # 只有当速度变化时，或者每隔一定时间发送一次心跳
        # 这里为了简单，我们每 100ms 发送一次速度指令
        
        # 1. 构建 Payload (两个 float: linear_x, angular_z)
        # struct.pack: 'f' 代表 float (4字节), '<' 代表小端模式
        payload = struct.pack('<ff', target_vx, target_vw)
        
        # 2. 构建完整帧
        # 头(2) + 功能(1) + 长度(1) + Payload + 校验(1)
        # ID = 0x10 (设置速度)
        frame_header = b'\xAA\x55'
        frame_func = bytes([CMD_SPEED])
        frame_len = bytes([len(payload)])
        
        # 计算校验 (Func + Len + Payload)
        data_to_check = frame_func + frame_len + payload
        checksum = bytes([calc_checksum(data_to_check)])
        
        packet = frame_header + data_to_check + checksum
        
        try:
            ser.write(packet)
            # print(f"Sent Speed: X={target_vx:.2f} Z={target_vw:.2f}")
        except Exception as e:
            print(f"Serial Write Error: {e}")
            break
            
        time.sleep(0.1) # 10Hz 发送频率

# ================= 接收解析线程 =================
def receive_thread(ser):
    print("[RX] Receiving thread started...")
    
    state = 0 # 0:WaitAA, 1:Wait55, 2:WaitID, 3:WaitLen, 4:WaitPayload, 5:WaitCRC
    frame_func = 0
    frame_len = 0
    payload_buffer = b''
    check_sum_calc = 0
    
    while running:
        try:
            # 读取 1 个字节
            byte = ser.read(1)
            if not byte: continue
            
            val = ord(byte) # 转成整数
            
            # --- 状态机 ---
            if state == 0: # Wait Header 1
                if val == 0xAA: state = 1
                
            elif state == 1: # Wait Header 2
                if val == 0x55: 
                    state = 2
                    check_sum_calc = 0 # 重置校验和
                elif val == 0xAA: state = 1 # 特殊情况 AA AA
                else: state = 0
                
            elif state == 2: # Wait Func ID
                frame_func = val
                check_sum_calc += val
                state = 3
                
            elif state == 3: # Wait Len
                frame_len = val
                check_sum_calc += val
                payload_buffer = b''
                if frame_len == 0: state = 5 # 无 Payload，直接校验
                else: state = 4
                
            elif state == 4: # Read Payload
                payload_buffer += byte
                check_sum_calc += val
                if len(payload_buffer) == frame_len:
                    state = 5
                    
            elif state == 5: # Check CRC
                received_crc = val
                final_crc = check_sum_calc & 0xFF
                
                if received_crc == final_crc:
                    # --- 校验通过，开始解包 ---
                    parse_packet(frame_func, payload_buffer)
                else:
                    print(f"CRC Error! Calc:{final_crc:02X} Recv:{received_crc:02X}")
                
                state = 0 # 回到开头
                
        except Exception as e:
            print(f"Serial Read Error: {e}")
            break

def parse_packet(func, data):
    # 根据你的 usart_task.c 里的协议解包
    if func == 0x10: # IMU 数据 (3 floats: ax, ay, az)
        if len(data) == 12:
            ax, ay, az = struct.unpack('<fff', data)
            print(f"\r[IMU] Accel: X={ax:6.2f} Y={ay:6.2f} Z={az:6.2f}", end='')
            
    elif func == 0x20: # 电机数据 (2 floats: L, R)
        if len(data) == 8:
            sp_l, sp_r = struct.unpack('<ff', data)
            print(f" | [Motor] L={sp_l:6.2f} R={sp_r:6.2f}", end='')

# ================= 主程序 =================
def main():
    global running, target_vx, target_vw
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE}")
    except Exception as e:
        print(f"Could not open port: {e}")
        return

    # 启动收发线程
    t_rx = threading.Thread(target=receive_thread, args=(ser,))
    t_tx = threading.Thread(target=send_thread, args=(ser,))
    t_rx.start()
    t_tx.start()
    
    print("\n=== Robot Control Panel ===")
    print(" [W] Accelerate")
    print(" [S] Decelerate / Reverse")
    print(" [A] Turn Left")
    print(" [D] Turn Right")
    print(" [Space] STOP")
    print(" [Q] Quit")
    print("===========================\n")

    # 简单的键盘监听循环 (Windows/Linux 通用性较差，这里用 input 模拟简单指令)
    # 为了更好的交互，可以使用 keyboard 库，但这里为了不依赖库，使用简易版
    try:
        import msvcrt # Windows Only for key detection
        while running:
            if msvcrt.kbhit():
                key = msvcrt.getch().decode('utf-8').lower()
                
                if key == 'w': target_vx += 0.1
                elif key == 's': target_vx -= 0.1
                elif key == 'a': target_vw += 0.5
                elif key == 'd': target_vw -= 0.5
                elif key == ' ': target_vx = 0; target_vw = 0
                elif key == 'q': running = False
                
                # 限制幅度
                target_vx = max(-2.0, min(2.0, target_vx))
                
                print(f"\n>> Cmd: Vx={target_vx:.1f}, Vw={target_vw:.1f}")
                
            time.sleep(0.05)
            
    except ImportError:
        # Linux / Mac 用户如果没有 msvcrt
        print("Non-Windows system detected. Input mode:")
        while running:
            cmd = input("Enter (w/s/a/d/q): ")
            if cmd == 'w': target_vx += 0.5
            elif cmd == 's': target_vx -= 0.5
            elif cmd == 'q': running = False
            # ...

    running = False
    ser.close()
    t_rx.join()
    t_tx.join()
    print("Bye!")

if __name__ == "__main__":
    main()