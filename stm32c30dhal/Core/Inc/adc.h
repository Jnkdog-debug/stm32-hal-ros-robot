/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "sys.h"
#include "system.h"

/* USER CODE BEGIN Includes */
#define Battery_Ch    8 //Battery voltage, ADC channel 14 //电池电压，ADC通道14
#define Potentiometer 9  //Potentiometer, ADC channel 8 //电位器，ADC通道8

#define TSL_SI    PCout(4)   //SI  
#define TSL_CLK   PAout(5)   //CLK 
/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN Private defines */
void HAL_ADC_MsInit(ADC_HandleTypeDef* adcHandle);
/* USER CODE END Private defines */

void MX_ADC1_Init(void);

/* USER CODE BEGIN Prototypes */
u16 Get_Adc(u8 ch);
float Get_battery_volt(void);
void Dly_us(void);
void RD_TSL(void); 
void Find_CCD_Median (void);
void CCD_ADC_Mode_Config(void);
void CCD_GPIO_Config(void);
void CCD_Init(void);
void ELE_ADC_Mode_Config(void);
void ELE_ADC_Init(void);
void ELE_GPIO_Config(void);

u16 Get_adc_Average(u8 chn, u8 times);
extern float Voltage,Voltage_Count,Voltage_All; 
extern u16 CCD_Median,CCD_Threshold, ADV[128];
extern int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor,sum;
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

