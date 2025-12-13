import serial
import struct
import threading
import time
import sys

# ================= 配置区 =================
# Linux 常用端口，请根据实际情况修改
SERIAL_PORT = '/dev/ttyACM0'  
BAUD_RATE = 115200

# ================= 协议定义 =================
HEADER = b'\xAA\x55'
CMD_SPEED  = 0x10

# 全局状态
running = True
target_vx = 0.0 # 线速度 (前后)
target_vw = 0.0 # 角速度 (旋转)

# 限制参数
MAX_PWM = 100.0
STEP_VAL = 10.0

# ================= CRC 校验 =================
def calc_checksum(data_bytes):
    return sum(data_bytes) & 0xFF

# ================= 发送线程 (含差速解算) =================
def send_thread(ser):
    global target_vx, target_vw
    
    print("[TX] Sending thread started...")
    
    while running:
        # --- 1. 差速运动学解算 (核心修改区) ---
        
        # 原始计算：假设两个电机同向安装
        # 左轮 = 线速度 - 角速度
        # 右轮 = 线速度 + 角速度
        
        raw_l = target_vx - target_vw
        raw_r = target_vx + target_vw
        
        # --- 【这里修改了！】适配你的镜像电机 ---
        # 左轮保持不变
        motor_l = raw_l
        
        # 右轮取反！(因为你的右轮装反了，正数代表后退，负数代表前进)
        motor_r = -raw_r  
        
        # 结果验证：
        # 如果前进 (vx=10, vw=0) -> raw_l=10, raw_r=10 -> 发送 L=10, R=-10 (符合你的要求)
        # 如果左转 (vx=0, vw=10) -> raw_l=-10, raw_r=10 -> 发送 L=-10, R=-10 
        #   (左轮-10后退，右轮-10前进 -> 也就是顺时针/逆时针 原地旋转)
        
        # --- 2. 限幅 ---
        motor_l = max(-MAX_PWM, min(MAX_PWM, motor_l))
        motor_r = max(-MAX_PWM, min(MAX_PWM, motor_r))
        
        # --- 3. 打包 ---
        payload = struct.pack('<ff', motor_l, motor_r)
        
        # 构建完整帧
        frame_header = b'\xAA\x55'
        frame_func = bytes([CMD_SPEED])
        frame_len = bytes([len(payload)])
        
        data_to_check = frame_func + frame_len + payload
        checksum = bytes([calc_checksum(data_to_check)])
        
        packet = frame_header + data_to_check + checksum
        
        try:
            ser.write(packet)
        except Exception as e:
            print(f"Serial Write Error: {e}")
            break
            
        time.sleep(0.1)

# ================= 接收解析线程 =================
def receive_thread(ser):
    print("[RX] Receiving thread started...")
    
    state = 0 
    frame_func = 0
    frame_len = 0
    payload_buffer = b''
    check_sum_calc = 0
    
    while running:
        try:
            byte = ser.read(1)
            if not byte: continue
            val = ord(byte)
            
            # --- 状态机 (校验和不含头) ---
            if state == 0: 
                if val == 0xAA: state = 1
            elif state == 1: 
                if val == 0x55: 
                    state = 2
                    check_sum_calc = 0 
                elif val == 0xAA: state = 1 
                else: state = 0
            elif state == 2: 
                frame_func = val
                check_sum_calc += val
                state = 3
            elif state == 3: 
                frame_len = val
                check_sum_calc += val
                payload_buffer = b''
                if frame_len == 0: state = 5 
                else: state = 4
            elif state == 4: 
                payload_buffer += byte
                check_sum_calc += val
                if len(payload_buffer) == frame_len:
                    state = 5
            elif state == 5: 
                received_crc = val
                final_crc = check_sum_calc & 0xFF
                
                if received_crc == final_crc:
                    parse_packet(frame_func, payload_buffer)
                # else:
                #     print(f"CRC Error!", flush=True) # 调试时可打开
                
                state = 0 
                
        except Exception as e:
            print(f"Serial Read Error: {e}")
            break

def parse_packet(func, data):
    # 记得加 flush=True 防止卡死
    if func == 0x10: 
        if len(data) == 12:
            ax, ay, az = struct.unpack('<fff', data)
            print(f"\r[IMU] Z={az:4.2f}", end='', flush=True)
            
    elif func == 0x20: 
        if len(data) == 8:
            sp_l, sp_r = struct.unpack('<ff', data)
            # 只显示一位小数，看起来清爽点
            print(f" | [M] L={sp_l:6.0f} R={sp_r:6.0f}", end='', flush=True)

# ================= 主程序 =================
def main():
    global running, target_vx, target_vw
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"Connected to {SERIAL_PORT}")
    except Exception as e:
        print(f"Error: {e}")
        return

    t_rx = threading.Thread(target=receive_thread, args=(ser,))
    t_tx = threading.Thread(target=send_thread, args=(ser,))
    t_rx.start()
    t_tx.start()
    
    print("\n=== Linux Robot Control ===")
    print(" Enter command and press ENTER:")
    print(" w: Forward (+10)")
    print(" s: Backward (-10)")
    print(" a: Left (+10 spin)")
    print(" d: Right (-10 spin)")
    print(" x: STOP (0,0)")
    print(" q: Quit")
    print("===========================\n")

    # Linux 简易输入循环
    while running:
        try:
            cmd = input().lower().strip() # 等待用户输入并回车
            
            if cmd == 'w': target_vx += STEP_VAL
            elif cmd == 's': target_vx -= STEP_VAL
            elif cmd == 'a': target_vw += STEP_VAL # 左转增加角速度
            elif cmd == 'd': target_vw -= STEP_VAL # 右转减小角速度
            elif cmd == 'x': target_vx = 0; target_vw = 0
            elif cmd == 'q': running = False
            
            # 限制幅度
            target_vx = max(-MAX_PWM, min(MAX_PWM, target_vx))
            target_vw = max(-MAX_PWM, min(MAX_PWM, target_vw))
            
            # 打印当前目标状态
            print(f"\n[SET] Vx={target_vx} Vw={target_vw}")
            
        except EOFError:
            break

    running = False
    ser.close()
    t_rx.join()
    t_tx.join()
    print("Bye!")

if __name__ == "__main__":
    main()