#include "encodermotor.h"
#include <stdint.h>



MotorControl_t Motor_A = {
    .pwm_timo1 = &htim10,
    .pwm_channel1 = TIM_CHANNEL_1,
    .pwm_timo2 = &htim11,
    .pwm_channel2 = TIM_CHANNEL_1,
    .encoder_tim = &htim2,
    .kp = 0.5f,
    .ki = 0.2f,
    .kd = 0.05f
};


// 电机 B: PWM (TIM9_CH1/CH2), 编码器 (TIM3)
MotorControl_t Motor_B = {
    .pwm_timo1 = &htim9, 
    .pwm_channel1 = TIM_CHANNEL_1,
    .pwm_timo2 = &htim9, 
    .pwm_channel2 = TIM_CHANNEL_2,
    .encoder_tim = &htim3, 
    .kp = 0.5f,
    .ki = 0.2f,
    .kd = 0.05f
};

// 电机 C: PWM (TIM1_CH1/CH2), 编码器 (TIM4)
MotorControl_t Motor_C = {
    .pwm_timo1 = &htim1, 
    .pwm_channel1 = TIM_CHANNEL_1,
    .pwm_timo2 = &htim1, 
    .pwm_channel2 = TIM_CHANNEL_2,
    .encoder_tim = &htim4, 
    .kp = 0.5f,
    .ki = 0.2f,
    .kd = 0.05f
};

// 电机 D: PWM (TIM1_CH3/CH4), 编码器 (TIM5)
MotorControl_t Motor_D = {
    .pwm_timo1 = &htim1, 
    .pwm_channel1 = TIM_CHANNEL_3,
    .pwm_timo2 = &htim1, 
    .pwm_channel2 = TIM_CHANNEL_4,
    .encoder_tim = &htim5, 
    .kp = 0.5f,
    .ki = 0.2f,
    .kd = 0.05f
};

void Motor_Init(MotorControl_t* motor)
{
    
    
    HAL_TIM_PWM_Start(motor->pwm_timo1, motor->pwm_channel1);
    HAL_TIM_PWM_Start(motor->pwm_timo2, motor->pwm_channel2);

    __HAL_TIM_SET_COMPARE(motor->pwm_timo1, motor->pwm_channel1, 0);
    __HAL_TIM_SET_COMPARE(motor->pwm_timo2, motor->pwm_channel2, 0);
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
    motor->pid_integral = 0.0f;
    motor->pid_last_error = 0.0f;

    HAL_TIM_Encoder_Start(motor->encoder_tim, TIM_CHANNEL_ALL);
    HAL_TIM_Base_Start_IT(&htim6);  // ⚠️ 必须调用，否则不会触发中断
}

/*读取编码器数值*/
int32_t Motor_Encoder_Read(MotorControl_t* motor) {
    int32_t encoder_value = __HAL_TIM_GET_COUNTER(motor->encoder_tim);
    int32_t delta = encoder_value - motor->encoder_last;
    motor->encoder_last = encoder_value;

    // 计算当前速度（RPM）
    motor->duty_rpm = delta;  // 根据编码器的计数值计算速度
    return motor->duty_rpm;
}



// 配置参数
#define ENCODER_PPR      13       // 编码器原始 PPR
#define MULTIPLIER       4        // 倍频数（你现在是 TIM_ENCODERMODE_TI12）
#define GEAR_RATIO       20.0f    // 减速比
#define SAMPLE_TIME_S    0.1f     // 采样时间，单位秒

void Motor_Encoder_SpeedUpdate(MotorControl_t* motor)
{
    uint32_t now = __HAL_TIM_GET_COUNTER(motor->encoder_tim);
    int32_t delta = (int32_t)(now - motor->encoder_last);
    motor->encoder_last = now;

    // 一圈多少个计数？
    const  float pulses_per_rev = ENCODER_PPR * MULTIPLIER;  // 13 * 4 = 52
    float rpm = (delta / pulses_per_rev) * (60.0f / SAMPLE_TIME_S);  // 电机原轴 RPM

    // 如果想要的是减速后的输出轴速度，需除以减速比
    motor->duty_rpm = rpm / GEAR_RATIO;

    // printf("Motor Output RPM: %.3f\r\n", motor->duty_rpm);
}

/*速度pid的实现*/
float Motor_PID_Calculate(MotorControl_t* motor, int32_t target_rpm) {
    // 计算误差
    float error = target_rpm - motor->duty_rpm;

    // 积分
    motor->pid_integral += error;

    // 微分
    float derivative = error - motor->pid_last_error;
    motor->pid_last_error = error;

    // PID输出
    float output = (motor->kp * error) + (motor->ki * motor->pid_integral) + (motor->kd * derivative);

    return output;
}

/*编码器电机的pid速度控制*/
void Motor_PID_Control(MotorControl_t* motor) {
    // 定期更新编码器速度
    Motor_Encoder_SpeedUpdate(motor);

    // 计算 PID 输出
    float pid_output = Motor_PID_Calculate(motor, motor->target_rpm);

    // 将 PID 输出转换为 PWM 值
    int16_t pwm_duty = (int16_t)pid_output;

    // 限制 PWM 范围
    if (pwm_duty > 1000) pwm_duty = 1000;  // 假设最大 PWM 值为 1000
    if (pwm_duty < -1000) pwm_duty = -1000;

    // 设置 PWM
    Motor_SetPWM(motor, pwm_duty);

    // 打印调试信息
    printf("Target RPM: %.3f, Current RPM: %.3f, PID Output: %.3f, PWM Duty: %d\r\n",
           motor->target_rpm, motor->duty_rpm, pid_output, pwm_duty);
}