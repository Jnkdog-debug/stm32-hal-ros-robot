import serial
import struct
import time

class RobotBridge:
    def __init__(self, port='/dev/ttyACM0', baud=115200):
        try:
            self.ser = serial.Serial(port, baud, timeout=0.5)
            print(f"✅ 已连接到: {port} @ {baud}")
        except Exception as e:
            print(f"❌ 连接失败: {e}")
            exit(1)

        self.HEADER = b'\xAA\x55'

    def read_data(self):
        """ 解析状态机：读取下位机上报的数据 """
        if self.ser.in_waiting < 5: # 最小长度: 头(2) + 功能(1) + 长度(1) + 校验(1)
            return

        # 1. 找帧头
        if self.ser.read(1) != b'\xAA': return
        if self.ser.read(1) != b'\x55': return

        # 2. 读功能位和长度
        func = ord(self.ser.read(1))
        data_len = ord(self.ser.read(1))

        # 3. 读载荷内容
        payload = self.ser.read(data_len)
        if len(payload) != data_len: return

        # 4. 校验
        received_checksum = ord(self.ser.read(1))
        calculated_checksum = (func + data_len + sum(payload)) & 0xFF

        if received_checksum == calculated_checksum:
            self.parse_payload(func, payload)
        else:
            print("⚠️ 校验错误!")

    def parse_payload(self, func, data):
        if func == 0x10: 
            if len(data) == 12:
                ax, ay, az = struct.unpack('<fff', data)
                # \r 回到行首，end='' 不换行，这样数据只会在这一行跳动
                print(f"\r📍 [IMU] X:{ax:>7.2f} Y:{ay:>7.2f} Z:{az:>7.2f}          ", end='', flush=True)

        elif func == 0x20:
            if len(data) == 8:
                v_l, v_r = struct.unpack('<ff', data)
                # 增加一些空格覆盖旧字符
                print(f" | 🏎️  [Wheel] L:{v_l:>7.2f} R:{v_r:>7.2f}          ", end='', flush=True)

    def send_speed(self, l_speed, r_speed):
        """ 发送控制指令给 STM32 """
        payload = struct.pack('<ff', float(l_speed), float(r_speed))
        func = 0x10
        data_len = len(payload)
        checksum = (func + data_len + sum(payload)) & 0xFF
        
        packet = self.HEADER + bytes([func, data_len]) + payload + bytes([checksum])
        self.ser.write(packet)

# --- 使用示例 ---
if __name__ == "__main__":
    robot = RobotBridge(port='/dev/ttyACM0') # 如果是香橙派，确认是 ACM0 还是 USB0
    
    try:
        while True:
            robot.read_data()
            # robot.send_speed(10.0, -10.0) # 测试发送
            time.sleep(0.01) # 100Hz 循环
    except KeyboardInterrupt:
        print("\n退出程序")