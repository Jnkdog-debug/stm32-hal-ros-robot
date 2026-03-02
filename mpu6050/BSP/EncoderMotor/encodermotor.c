#include "encodermotor.h"
#include <stdint.h>



// ==========================================
// 麦克纳姆轮底盘：四轮电机硬件资源全映射
// ==========================================

// 1. 电机 A (左前轮 FL)
// 硬件分配: PWM -> TIM10_CH1 / TIM11_CH1, 编码器 -> TIM2 PA15 PB3
MotorControl_t Motor_A = {
    .pwm_timo1 = &htim11, 
    .pwm_channel1 = TIM_CHANNEL_1,
    .pwm_timo2 = &htim10, 
    .pwm_channel2 = TIM_CHANNEL_1,
    .encoder_tim = &htim2, 
    .encoder_direction = 1,   // 根据实际接线调整 1 或 -1
    .kp = 5.0f, .ki = 0.05f, .kd = 0.1f
};

// 2. 电机 B (右前轮 FR)
// 硬件分配: PWM -> TIM9_CH1 / TIM9_CH2, 编码器 -> TIM3 PB4 PB5
MotorControl_t Motor_B = {
    .pwm_timo1 = &htim9, 
    .pwm_channel1 = TIM_CHANNEL_2,
    .pwm_timo2 = &htim9, 
    .pwm_channel2 = TIM_CHANNEL_1,
    .encoder_tim = &htim3, 
    .encoder_direction =-1,   
    .kp = 5.0f, .ki = 0.05f, .kd = 0.1f
};

// 3. 电机 C (左后轮 RL)
// 硬件分配: PWM -> TIM1_CH1 / TIM1_CH2, 编码器 -> TIM4 PB6 PB7
MotorControl_t Motor_C = {
    .pwm_timo1 = &htim1, 
    .pwm_channel1 = TIM_CHANNEL_1,
    .pwm_timo2 = &htim1, 
    .pwm_channel2 = TIM_CHANNEL_2,
    .encoder_tim = &htim4, 
    .encoder_direction =1,   
    .kp = 5.0f, .ki = 0.05f, .kd = 0.1f
};

// 4. 电机 D (右后轮 RR)
// 硬件分配: PWM -> TIM1_CH3 / TIM1_CH4, 编码器 -> TIM5 PA0 PA1
MotorControl_t Motor_D = {
    .pwm_timo1 = &htim1, 
    .pwm_channel1 = TIM_CHANNEL_3,
    .pwm_timo2 = &htim1, 
    .pwm_channel2 = TIM_CHANNEL_4,
    .encoder_tim = &htim5, 
    .encoder_direction =-1,   
    .kp = 5.0f, .ki = 0.05f, .kd = 0.1f
};

// 建议：为了方便后续在 FreeRTOS 的 motorTask 中用 for 循环遍历控制，
// 可以把它们装进一个数组里！
MotorControl_t* Mecanum_Motors[4] = {&Motor_A, &Motor_B, &Motor_C, &Motor_D};

void Motor_Init(MotorControl_t* motor)
{
    
    HAL_TIM_PWM_Start(motor->pwm_timo1, motor->pwm_channel1);
    HAL_TIM_PWM_Start(motor->pwm_timo2, motor->pwm_channel2);

    __HAL_TIM_SET_COMPARE(motor->pwm_timo1, motor->pwm_channel1, 0);
    __HAL_TIM_SET_COMPARE(motor->pwm_timo2, motor->pwm_channel2, 0);
    motor->last_raw_value = (uint16_t)__HAL_TIM_GET_COUNTER(motor->encoder_tim);
}


void Motor_SetPWM(MotorControl_t* motor, int16_t duty) {
    uint16_t pwm_val = (duty < 0) ? -duty : duty;

    if (duty < 0) {
        __HAL_TIM_SET_COMPARE(motor->pwm_timo1, motor->pwm_channel1, 0);
        __HAL_TIM_SET_COMPARE(motor->pwm_timo2, motor->pwm_channel2, pwm_val);
    } else if (duty > 0) {
        __HAL_TIM_SET_COMPARE(motor->pwm_timo1, motor->pwm_channel1, pwm_val);
        __HAL_TIM_SET_COMPARE(motor->pwm_timo2, motor->pwm_channel2, 0);
    } else {  // duty == 0
        __HAL_TIM_SET_COMPARE(motor->pwm_timo1, motor->pwm_channel1, 0);
        __HAL_TIM_SET_COMPARE(motor->pwm_timo2, motor->pwm_channel2, 0);
    }
}


/*编码器实现*/

void Motor_Encoder_Init(MotorControl_t* motor) {
    motor->encoder_last = 0;
    motor->duty_rpm = 0;
    motor->target_rpm = 0;
    motor->pid_prev_error = 0.0f;
    motor->pid_last_error = 0.0f;
    motor->pid_integral = 0.0f;
    motor->pid_out = 0.0f;

    HAL_TIM_Encoder_Start(motor->encoder_tim, TIM_CHANNEL_ALL);
}






// 修正后的配置参数
#define ENCODER_PPR       13.0f   
#define MULTIPLIER        4.0f    
#define GEAR_RATIO        20.0f   
#define SAMPLE_TIME_S     0.01f   // 必须与 Task 的 10ms 匹配！

int32_t Motor_Encoder_Update(MotorControl_t* motor) {
    // 1. 获取硬件原始值
    uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(motor->encoder_tim);
    
    // 2. 计算差值
    int16_t delta = (int16_t)(now - motor->last_raw_value);
    
    // 3. 根据编码器方向调整脉冲值
    delta *= motor->encoder_direction;
    
    // 4. 更新总脉冲
    motor->total_count += delta;
    
    // 5. 计算当前 RPM
    // 逻辑：(脉冲数 / 每转总脉冲) / 时间 = 转/秒，再 * 60 = 转/分
    // 别忘了除以减速比才是轮子的输出转速
    float pulses_per_rev = ENCODER_PPR * MULTIPLIER * GEAR_RATIO;
    motor->duty_rpm = ((float)delta / pulses_per_rev) * (60.0f / SAMPLE_TIME_S);

    // 6. 更新旧值
    motor->last_raw_value = now;
    
    return motor->total_count;
}


int16_t Incremental_PID(MotorControl_t* motor, float target) {
    if (target == 0 && motor->duty_rpm == 0) {
        motor->pid_out = 0;
        motor->pid_last_error = 0;
        motor->pid_prev_error = 0;
        motor->pid_integral = 0;
        return 0;
    }

    float error = target - motor->duty_rpm;
    
    // 增量式PID计算：Δu(k) = Kp*[e(k)-e(k-1)] + Ki*e(k) + Kd*[e(k)-2*e(k-1)+e(k-2)]
    // 注意：积分项直接用 error，不要累积！
    // ✅ 正确：积分项只乘以当前误差，不累积
    float increment = motor->kp * (error - motor->pid_last_error) + 
                      motor->ki * error +  // ← 只是 error，不是 pid_integral
                      motor->kd * (error - 2 * motor->pid_last_error + motor->pid_prev_error);
                      
    motor->pid_out += increment;
    
    // 更新误差历史
    motor->pid_prev_error = motor->pid_last_error;
    motor->pid_last_error = error;
    
    // --- 限幅对应 ARR=999 ---
    if (motor->pid_out > 950)  motor->pid_out = 950;
    if (motor->pid_out < -950) motor->pid_out = -950;
    
    return (int16_t)motor->pid_out;
}