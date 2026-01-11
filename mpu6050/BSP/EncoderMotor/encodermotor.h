#ifndef __ENCODER_MOTOR_H__
#define __ENCODER_MOTOR_H__

#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"



typedef struct {
    TIM_HandleTypeDef* pwm_timo1;
    TIM_HandleTypeDef* pwm_timo2;
    uint32_t pwm_channel1;
    uint32_t pwm_channel2;


    TIM_HandleTypeDef* encoder_tim;
    int8_t encoder_direction;  // 编码器方向标志: 1=正向, -1=反向

    int32_t encoder_last;
    int32_t total_count;    // 自开机以来的总脉冲数（考虑正反转）
    int16_t last_raw_value; // 记录上一次定时器的原始计数值
    float duty_rpm;     // 当前速度
    float target_rpm;    // 目标速度

    float kp, ki, kd;
    float pid_integral;
    float pid_last_error;
    float pid_prev_error;
    float pid_out;

} MotorControl_t;




void Motor_Init(MotorControl_t* motor);
void Motor_SetPWM(MotorControl_t* motor, int16_t duty);
void Motor_Encoder_Init(MotorControl_t* motor);
int32_t Motor_Encoder_Update(MotorControl_t* motor);
int16_t Incremental_PID(MotorControl_t* motor, float target);

#endif