import serial
import struct
import time
import threading
import sys
import math

# ================= 🤖 机器人物理参数配置 =================
# 轮子直径 (米)
WHEEL_DIAMETER = 0.047  
# 两个轮子中心的间距 (米)
WHEEL_BASE = 0.150      
# 右轮是否需要反向？ (True: 直行时 右轮给负数 / False: 直行时 右轮给正数)
# 因为你是相对安装，如果左轮向前是正，右轮向前通常需要是负
RIGHT_WHEEL_INVERSE = True 

# ================= 串口配置 =================
SERIAL_PORT = '/dev/ttyACM0'  
BAUD_RATE = 115200
# ===========================================

class RobotKinematics:
    """ 运动学解算类：负责将 m/s 转换为 RPM """
    def __init__(self, diameter, base, right_inverse=False):
        self.d = diameter
        self.l = base
        self.right_inverse = right_inverse

    def inverse_kinematics(self, linear_vel, angular_vel):
        """
        输入: 线速度(m/s), 角速度(rad/s)
        输出: 左轮RPM, 右轮RPM
        """
        # 1. 计算左右轮的线速度 (m/s)
        # v_left  = v - (w * L) / 2
        # v_right = v + (w * L) / 2
        vel_l = linear_vel - (angular_vel * self.l) / 2.0
        vel_r = linear_vel + (angular_vel * self.l) / 2.0

        # 2. 将线速度转换为 RPM
        # RPM = (Vel / (PI * D)) * 60
        rpm_l = (vel_l / (math.pi * self.d)) * 60.0
        rpm_r = (vel_r / (math.pi * self.d)) * 60.0

        # 3. 处理右轮反向问题
        if self.right_inverse:
            rpm_r = -rpm_r

        return rpm_l, rpm_r

class RobotDashboard:
    def __init__(self, port, baud):
        self.running = True
        self.ser = None
        self.kinematics = RobotKinematics(WHEEL_DIAMETER, WHEEL_BASE, RIGHT_WHEEL_INVERSE)
        
        try:
            self.ser = serial.Serial(port, baud, timeout=0.02)
            print(f"✅ 串口已打开: {port} @ {baud}")
            print(f"⚙️  参数加载: 轮径={WHEEL_DIAMETER}m, 轮距={WHEEL_BASE}m")
            if RIGHT_WHEEL_INVERSE:
                print("⚠️  注意: 已启用右轮反向模式 (Right Wheel Inverted)")
        except Exception as e:
            print(f"❌ 串口打开失败: {e}")
            sys.exit(1)

        self.t_recv = threading.Thread(target=self._receive_loop, daemon=True)
        self.t_recv.start()

    def _receive_loop(self):
        buffer = b''
        while self.running:
            try:
                if self.ser.in_waiting:
                    buffer += self.ser.read(self.ser.in_waiting)
                
                while b'\xAA\x55' in buffer:
                    idx = buffer.find(b'\xAA\x55')
                    buffer = buffer[idx:]
                    if len(buffer) < 4: break 
                    
                    func = buffer[2]
                    length = buffer[3]
                    total_frame_len = 4 + length + 1
                    if len(buffer) < total_frame_len: break 
                    
                    payload = buffer[4 : 4+length]
                    checksum_recv = buffer[4+length]
                    checksum_calc = (func + length + sum(payload)) & 0xFF
                    
                    if checksum_calc == checksum_recv:
                        self._parse_payload(func, payload)
                        
                    buffer = buffer[total_frame_len:]
                time.sleep(0.005)
            except Exception as e:
                print(f"\n[串口错误] {e}")
                time.sleep(1)

    def _parse_payload(self, func, data):
        """ 解析具体的数据包 """
        
        # --- 1. IMU 数据解析 (功能码 0x10) ---
        # 长度必须是 24 字节 (6个 float)
        if func == 0x10 and len(data) == 24:
            try:
                # 解析 6 个浮点数
                # ax, ay, az: 加速度
                # gx, gy, gz: 角速度
                ax, ay, az, gx, gy, gz = struct.unpack('<ffffff', data)
                
                # 打印显示 (只显示 Z轴角速度，因为这是机器人航向角最关键的数据)
                print(f"\r🧭 [IMU] Acc_X:{ax:>5.2f} | Gyro_Z:{gz:>6.2f} rad/s ", end='', flush=True)
            except struct.error:
                pass

        # --- 2. 电机数据解析 (功能码 0x20) ---
        # 新格式: 4 + 4 + 4 + 4 + 4 = 20 字节 (cnt_L, cnt_R, rpm_L, rpm_R, battery_voltage)
        elif func == 0x20 and len(data) == 20:
            try:
                cnt_l, cnt_r, rpm_l, rpm_r, battery_voltage = struct.unpack('<iiff f', data)
                
                # 电压显示：根据电压值显示不同的颜色/符号
                if battery_voltage > 12.0:
                    voltage_indicator = "🟢"  # 绿色：正常
                elif battery_voltage > 11.0:
                    voltage_indicator = "🟡"  # 黄色：警告
                else:
                    voltage_indicator = "🔴"  # 红色：低电压
                
                print(f" | 🏎️ [L] {rpm_l:>5.1f} [R] {rpm_r:>5.1f} RPM | {voltage_indicator} 电池: {battery_voltage:.2f}V ", end='', flush=True)
            except struct.error:
                pass
        
        # --- 2b. 电机数据解析 (旧格式兼容 - 功能码 0x20，仅16字节) ---
        elif func == 0x20 and len(data) == 16:
            try:
                cnt_l, cnt_r, rpm_l, rpm_r = struct.unpack('<iiff', data)
                print(f" | 🏎️ [L] {rpm_l:>5.1f} [R] {rpm_r:>5.1f} RPM   ", end='', flush=True)
            except struct.error:
                pass

    def send_target_rpm(self, l_rpm, r_rpm):
        """ 发送计算好的 RPM 给下位机 """
        try:
            header = b'\xAA\x55'
            func = 0x10
            payload = struct.pack('<ff', float(l_rpm), float(r_rpm))
            length = len(payload)
            checksum = (func + length + sum(payload)) & 0xFF
            packet = header + bytes([func, length]) + payload + bytes([checksum])
            self.ser.write(packet)
        except Exception as e:
            print(f"\n❌ 发送失败: {e}")

    def send_motion_command(self, v, w):
        """ 发送运动学命令 (m/s, rad/s) """
        rpm_l, rpm_r = self.kinematics.inverse_kinematics(v, w)
        # 限制最大 RPM 防止飞车 (假设最大 400 RPM)
        MAX_RPM = 900.0
        rpm_l = max(min(rpm_l, MAX_RPM), -MAX_RPM)
        rpm_r = max(min(rpm_r, MAX_RPM), -MAX_RPM)
        
        self.send_target_rpm(rpm_l, rpm_r)
        print(f"\n>>> 运动指令: v={v} m/s, w={w} rad/s  -->  L={rpm_l:.1f}, R={rpm_r:.1f} RPM")

    def close(self):
        self.running = False
        if self.ser:
            self.ser.close()

# ================= 主程序 =================
if __name__ == "__main__":
    dashboard = RobotDashboard(SERIAL_PORT, BAUD_RATE)
    
    print("\n" + "="*60)
    print("   🏎️  机器人运动学控制终端")
    print("   [输入格式] v w")
    print("   例如: 0.2 0    (前进 0.2m/s)")
    print("         0 1.0    (原地左转 1.0rad/s)")
    print("         0.2 0.5  (边走边转)")
    print("   [退出] q")
    print("="*60 + "\n")

    try:
        while True:
            user_input = input() 
            if user_input.lower() == 'q':
                break
            
            try:
                parts = user_input.split()
                if len(parts) >= 2:
                    v = float(parts[0])
                    w = float(parts[1])
                    dashboard.send_motion_command(v, w)
                elif len(parts) == 1:
                    # 如果只输入一个数，默认是直行
                    v = float(parts[0])
                    dashboard.send_motion_command(v, 0.0)
                else:
                    print("请输入: 线速度 角速度")
            except ValueError:
                print("\n❌ 格式错误，请输入数字")
                
    except KeyboardInterrupt:
        pass
    finally:
        dashboard.send_target_rpm(0, 0)
        dashboard.close()
        print("\n👋 安全停车")