#include "encodermotor.h"
#include <stdint.h>



MotorControl_t Motor_A = {
    .pwm_timo1 = &htim10,
    .pwm_channel1 = TIM_CHANNEL_1,
    .pwm_timo2 = &htim11,
    .pwm_channel2 = TIM_CHANNEL_1,
    .encoder_tim = &htim2,
    .kp = 1.0f,
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


void Motor_Encoder_SpeedUpdate(MotorControl_t* motor)
{
    int32_t encoder_value = __HAL_TIM_GET_COUNTER(motor->encoder_tim);
    int32_t delta = encoder_value - motor->encoder_last;
    
    // 处理溢出（编码器模式通常是 16 位）
    if (delta > 32768) delta -= 65536;
    else if (delta < -32768) delta += 65536;

    motor->encoder_last = encoder_value;

    // 计算转速（假设编码器 PPR = 1000，减速比 ratio = 1）
    float ppr = 1000.0f;
    float sample_time = 0.1f;  // 100ms
    motor->duty_rpm = delta / ppr * 60.0f / sample_time;
    printf("delta: %d\r\n", delta);
    // 打印结果
    printf("RPM: %.5f\r\n", motor->duty_rpm);
}
