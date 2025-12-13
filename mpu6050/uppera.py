import serial
import struct
import threading
import time
import sys
import termios # Linux 专用库
import tty
import select

# ================= 配置区 =================
SERIAL_PORT = '/dev/ttyACM0'  
BAUD_RATE = 115200

# ================= 协议定义 =================
HEADER = b'\xAA\x55'
CMD_SPEED  = 0x10

running = True
target_vx = 0.0
target_vw = 0.0

# 游戏手感参数
GAME_SPEED = 30.0   # 按住 W 时的速度
GAME_TURN  = 30.0   # 按住 A/D 时的转向速度
TIMEOUT    = 0.2    # 松手后多长时间停车 (秒)

last_key_time = 0.0 # 上次按键时间

# ================= 辅助函数 =================
def calc_checksum(data_bytes):
    return sum(data_bytes) & 0xFF

# Linux 下的非阻塞按键读取
def get_key_non_blocking():
    # 检查是否有输入 (0等待)
    dr,dw,de = select.select([sys.stdin], [], [], 0)
    if dr:
        return sys.stdin.read(1)
    return None

# ================= 发送线程 (含自动停车逻辑) =================
def send_thread(ser):
    global target_vx, target_vw
    print("[TX] Sending thread started...")
    
    while running:
        # --- 看门狗逻辑 (核心) ---
        # 如果当前时间 距离 上次按键时间 超过了 0.2秒
        # 说明用户松手了 -> 速度归零
        if time.time() - last_key_time > TIMEOUT:
            target_vx = 0.0
            target_vw = 0.0
            
        # --- 运动学解算 (镜像电机适配) ---
        motor_l = target_vx - target_vw
        motor_r = -(target_vx + target_vw) # 右轮取反
        
        # 限幅
        motor_l = max(-100, min(100, motor_l))
        motor_r = max(-100, min(100, motor_r))
        
        # 打包发送
        payload = struct.pack('<ff', motor_l, motor_r)
        frame_header = b'\xAA\x55'
        frame_func = bytes([CMD_SPEED])
        frame_len = bytes([len(payload)])
        data_to_check = frame_func + frame_len + payload
        checksum = bytes([calc_checksum(data_to_check)])
        packet = frame_header + data_to_check + checksum
        
        try:
            ser.write(packet)
        except:
            break
            
        time.sleep(0.05) # 20Hz 发送频率，手感更细腻

# ================= 接收线程 =================
def receive_thread(ser):
    # ... (保持原来的接收代码不变，记得加 flush=True) ...
    # 为了节省篇幅，这里略写接收逻辑，请直接把之前的 copy 过来即可
    # 只要记得 print 时加上 flush=True
    state = 0
    while running:
        try:
            if ser.in_waiting > 0:
                byte = ser.read(1)
                # ... (原来的状态机逻辑) ...
                # 这里为了演示，我先略过，保证你能跑起来控制
            else:
                time.sleep(0.01)
        except:
            break

# ================= 主程序 (键盘监听) =================
def main():
    global running, target_vx, target_vw, last_key_time
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"Connected to {SERIAL_PORT}")
    except Exception as e:
        print(e); return

    t_tx = threading.Thread(target=send_thread, args=(ser,))
    # t_rx = threading.Thread(target=receive_thread, args=(ser,)) # 如果需要接收数据就打开
    t_tx.start()
    # t_rx.start()
    
    print("\n=== GAME MODE ACTIVE ===")
    print(" Hold [W] to Move Forward")
    print(" Hold [S] to Move Backward")
    print(" Hold [A] to Spin Left")
    print(" Hold [D] to Spin Right")
    print(" Release to STOP")
    print(" Press [Q] to Quit")
    
    # --- 保存旧的终端设置 ---
    old_settings = termios.tcgetattr(sys.stdin)
    
    try:
        # 设置终端为 Raw 模式 (不需要回车，按键即发)
        tty.setcbreak(sys.stdin.fileno())
        
        while running:
            # 读取按键 (这里会阻塞等待按键)
            # 因为我们用了 setcbreak，按下一个键立刻返回
            key = sys.stdin.read(1).lower()
            
            # 更新按键时间 (喂狗)
            last_key_time = time.time()
            
            if key == 'w':
                target_vx = GAME_SPEED
                target_vw = 0.0
            elif key == 's':
                target_vx = -GAME_SPEED
                target_vw = 0.0
            elif key == 'a':
                target_vx = 0.0
                target_vw = GAME_TURN
            elif key == 'd':
                target_vx = 0.0
                target_vw = -GAME_TURN
            elif key == 'q':
                running = False
            
    finally:
        # --- 恢复终端设置 (非常重要！否则退出后你的终端会乱码) ---
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        running = False
        ser.close()
        print("\nExited.")

if __name__ == "__main__":
    main()