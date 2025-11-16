/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "system.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim1;

extern TIM_HandleTypeDef htim2;

extern TIM_HandleTypeDef htim3;

extern TIM_HandleTypeDef htim4;

extern TIM_HandleTypeDef htim5;

extern TIM_HandleTypeDef htim8;

extern TIM_HandleTypeDef htim9;

extern TIM_HandleTypeDef htim10;

extern TIM_HandleTypeDef htim11;

extern TIM_HandleTypeDef htim12;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_TIM5_Init(void);
void MX_TIM8_Init(void);
void MX_TIM9_Init(void);
void MX_TIM10_Init(void);
void MX_TIM11_Init(void);
void MX_TIM12_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */
#define PWMA1   TIM10->CCR1 
#define PWMA2   TIM11->CCR1 

#define PWMB1   TIM9->CCR1 
#define PWMB2   TIM9->CCR2

#define PWMC1   TIM1->CCR1  
#define PWMC2   TIM1->CCR2 

#define PWMD1   TIM1->CCR3 
#define PWMD2   TIM1->CCR4

#define EN     PDin(3)  

#define Servo_PWM  TIM8->CCR4
#define SERVO_INIT 1500  //Servo zero point //¶æ»úÁãµã
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

