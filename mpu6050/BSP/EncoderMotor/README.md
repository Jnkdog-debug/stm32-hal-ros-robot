基于 STM32 HAL 库封装的编码器电机控制库，支持正/反转控制、PWM 驱动、编码器测速与位置反馈、PID 控制接口，适用于 ROS 小车、智能硬件等需要闭环速度控制的电机场景。

# 📦 功能特性

    ✅ 支持 PWM + DIR 电机驱动接口（如 AT8236）

    ✅ 支持 TIM 编码器读取速度与位移

    ✅ 可配置 PID 控制器

    ✅ 实现正/反转控制

    ✅ 可拓展与 ROS（如通过串口、CAN、串口通信）

    ✅ 适配 STM32 HAL 驱动架构

    ✅ 适合 STM32F1/F4/F7 等系列芯片

# ⚙️ 硬件接线要求
接口	说明
PWM	连接到电机驱动 PWM 控制脚
DIR	控制电机正转/反转
ENCODER_A/B	编码器输出两路信号

建议使用 STM32 的 TIMx 编码器模式（Encoder Mode）连接编码器引脚。
# 🧱 类结构（C++）

class EncoderMotor {
public:
    EncoderMotor(TIM_HandleTypeDef* pwm_timer, uint32_t pwm_channel,
                 GPIO_TypeDef* dir_port, uint16_t dir_pin,
                 TIM_HandleTypeDef* encoder_timer);

    void setTargetSpeed(float speed_mps);
    void update(); // 每隔固定时间调用，用于PID更新
    float getCurrentSpeed();
    float getDistance();
    void resetEncoder();

    void setPID(float kp, float ki, float kd);

private:
    void setPWM(uint16_t duty);
    void updateEncoder();
    void computePID();

    // ... 内部成员变量略
};

# 🚀 快速上手
1️⃣ 初始化对象

EncoderMotor motor(&htim1, TIM_CHANNEL_1, GPIOA, GPIO_PIN_5, &htim2); // 例：TIM1输出PWM，PA5为方向引脚，TIM2为编码器输入
motor.setPID(1.0f, 0.2f, 0.1f);

2️⃣ 设置目标速度

motor.setTargetSpeed(0.3f); // 设置目标速度为 0.3 米每秒

3️⃣ 在定时器中定期调用更新

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        motor.update(); // 每 10ms 或 20ms 更新一次 PID
    }
}

# 📐 参数说明
参数	说明
speed_mps	速度，单位为 m/s
PWM duty	0 ~ 100（%），控制输出强度
direction pin	高电平正转，低电平反转
encoder count	编码器当前值，用于测速/位移
📘 典型应用

    小车左右轮电机闭环控制

    ROS 小车差速驱动模块

    自动导航机器人底盘

    PID 电机速度控制实验


记录：

使用电机代码时要加hal_dealy 否则电机反应不过来，驱动会保护的

例如：
  Motor_SetPWM(&Motor_A, 25);
  HAL_Delay(1000);
  Motor_SetPWM(&Motor_A, -15);
  HAL_Delay(1000);
