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

    int32_t encoder_last;
    float duty_rpm;     // 当前速度
    float target_rpm;    // 目标速度

    float kp, ki, kd;
    float pid_integral;
    float pid_last_error;
} MotorControl_t;
extern MotorControl_t Motor_A ;

void Motor_Init(MotorControl_t* motor);
void Motor_SetPWM(MotorControl_t* motor, int16_t duty);
void Motor_Encoder_Init(MotorControl_t* motor);
int32_t Motor_Encoder_Read(MotorControl_t* motor);
void Motor_Encoder_SpeedUpdate(MotorControl_t* motor);


#endif