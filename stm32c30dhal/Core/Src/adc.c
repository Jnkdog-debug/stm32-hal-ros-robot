/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
/* Includes ------------------------------------------------------------------*/
#include "adc.h"
#include "main.h"

/* USER CODE BEGIN 0 */
float Voltage,Voltage_Count,Voltage_All; //Variables related to battery voltage sampling //电池电压采样相关的变量  
const float Revise=1.03;

//CCD相关变量
u16 CCD_Median,CCD_Threshold,ADV[128]={0}; 

//电磁巡线相关变量																										
int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor,sum; 
/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  HAL_ADC_Start(&hadc1);//开启adc校准
  /* USER CODE END ADC1_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

//    __HAL_RCC_GPIOA_CLK_ENABLE();
//    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA4     ------> ADC1_IN4
    PA5     ------> ADC1_IN5
    PC4     ------> ADC1_IN14
    PB0     ------> ADC1_IN8
    PB1     ------> ADC1_IN9
    */
//    GPIO_InitStruct.Pin = GPIO_PIN_4;
//    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//		
//		GPIO_InitStruct.Pin = GPIO_PIN_5;
//    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//    GPIO_InitStruct.Pin = GPIO_PIN_4;
//    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ADC1 interrupt Init */
//    HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
//    HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA4     ------> ADC1_IN4
    PA5     ------> ADC1_IN5
    PC4     ------> ADC1_IN14
    PB0     ------> ADC1_IN8
    PB1     ------> ADC1_IN9
    */
//    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4|GPIO_PIN_5);

//    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0|GPIO_PIN_1);

    /* ADC1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
u16 Get_Adc(u8 ch)   
{
	u16 result;
	ADC_ChannelConfTypeDef sConfig;//通道初始化
	sConfig.Channel=ch;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;		//采用周期239.5周期
	sConfig.Rank = 1;
	HAL_ADC_ConfigChannel(&hadc1,&sConfig);		
	
	HAL_ADC_Start(&hadc1);								//启动转换
	HAL_ADC_PollForConversion(&hadc1,30);				//等待转化结束
	if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
	{
		result=HAL_ADC_GetValue(&hadc1);	//返回最近一次ADC1规则组的转换结果
	}
	return result;	
}

/**************************************************************************
Function: Collect multiple ADC values to calculate the average function
Input   : ADC channels and collection times
Output  : AD conversion results
函数功能：采集多次ADC值求平均值函数
入口参数：ADC通道和采集次数
返 回 值：AD转换结果
**************************************************************************/
u16 Get_adc_Average(u8 chn, u8 times)
{
  u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(chn);
		delay_ms(5);
	}
	return temp_val/times;
}

/**************************************************************************
Function: Read the battery voltage
Input   : none
Output  : Battery voltage in mV
函数功能：读取电池电压 
入口参数：无
返回  值：电池电压，单位mv
**************************************************************************/
float Get_battery_volt(void)   
{  
		int Volt;
//    HAL_ADC_Start(&hadc1);                                //开启ADC
//    HAL_ADC_PollForConversion(&hadc1,10);                 //轮询转换
	  Volt = Get_Adc(8)*3.3*11*1.03/4096;	//电阻分压，具体根据原理图简单分析可以得到	
	  return Volt;	                                      	//返回最近一次ADC1规则组的转换结果
}


/**************************************************************************
Function: CCD_ADC_Mode_Config
Input   : none
Output  : none
函数功能：初始化CCD巡线ADC
入口参数: 无
返回  值：无
**************************************************************************/	 	
void CCD_ADC_Mode_Config(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = 4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  HAL_ADC_Start(&hadc1);								//启动转换

}

/**************************************************************************
Function: CCD_GPIO_Config
Input   : none
Output  : none
函数功能：初始化CCD巡线GPIO
入口参数: 无
返回  值：无
**************************************************************************/	 	
void CCD_GPIO_Config(void)
{
	 GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	// __HAL_RCC_ADC1_CLK_ENABLE();
	// 打开CCD端口时钟
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	// CLK,SI配置为输出	
	  GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);		
	
//	  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4|GPIO_PIN_5);

//    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4);
//	
}

/**************************************************************************
Function: CCD_Init
Input   : none
Output  : none
函数功能：初始化CCD巡线
入口参数: 无
返回  值：无
**************************************************************************/	 	
void CCD_Init(void)
{
	CCD_GPIO_Config();
	CCD_ADC_Mode_Config();

}


/**************************************************************************
Function: ELE_ADC_Mode_Config
Input   : none
Output  : none
函数功能：初始化电磁巡线ADC
入口参数: 无
返回  值：无
**************************************************************************/	 	

void ELE_ADC_Mode_Config(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = 4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	
	sConfig.Channel = 5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	
	sConfig.Channel = 14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  HAL_ADC_Start(&hadc1);								//启动转换
}


void ELE_GPIO_Config(void)
{
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		
		//__HAL_RCC_ADC1_CLK_ENABLE();
		// 打开CCD端口时钟
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOC_CLK_ENABLE();
		
	  GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

//    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4|GPIO_PIN_5);

//    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4);

}

/**************************************************************************
Function: ELE_ADC_Init
Input   : none
Output  : none
函数功能：初始化电磁巡线ADC
入口参数: 无
返回  值：无
**************************************************************************/	 	
//电磁巡线初始化
void ELE_ADC_Init(void)
{
	ELE_GPIO_Config();
	//ELE_ADC_Mode_Config();
}

/**************************************************************************
函数功能：延时
入口参数：无
返回  值：无
**************************************************************************/
void Dly_us(void)
{
   int ii;    
   for(ii=0;ii<30;ii++); 
}
/**************************************************************************
函数功能：CCD数据采集
入口参数：无
返回  值：无
**************************************************************************/
 void RD_TSL(void) 
{
  u8 i=0,tslp=0;
  TSL_CLK=0;
  TSL_SI=0; 
  Dly_us();
      
  TSL_SI=1;
  Dly_us();	
	
  TSL_CLK=0;
  Dly_us();
	
      
  TSL_CLK=1;
	Dly_us(); 
	
  TSL_SI=0;
  Dly_us();
  for(i=0;i<128;i++)
  { 
    TSL_CLK=0; 
    Dly_us();  //调节曝光时间

    ADV[tslp]=(Get_Adc(4))>>4;
    ++tslp;
    TSL_CLK=1;
     Dly_us();

  }  
}
/**************************************************************************
函数功能：线性CCD取中值
入口参数：无
返回  值：无
**************************************************************************/
void  Find_CCD_Median (void)
{ 
	 static u16 i,j,Left,Right,Last_CCD_Median;
	 static u16 value1_max,value1_min;
	 //阈值说明：CCD采集回来的128个数据，每个数据单独与阈值进行比较，比阈值大为白色，比阈值小为黑色
	 //动态阈值算法，读取每次采集数据的最大和最小值的平均数作为阈值 
	 value1_max=ADV[0];  
   for(i=5;i<123;i++)   //两边各去掉5个点
     {
       if(value1_max<=ADV[i])
       value1_max=ADV[i];
     }
	  value1_min=ADV[0];  //最小值
    for(i=5;i<123;i++) 
     {
       if(value1_min>=ADV[i])
       value1_min=ADV[i];
     }
   CCD_Threshold =(value1_max+value1_min)/2;	  //计算出本次中线提取的阈值
		 
	 for(i = 5;i<118; i++)   //寻找左边跳变沿，连续三个白像素后连续三个黑像素判断左边跳变沿
	 {
		 if(ADV[i]>CCD_Threshold &&ADV[i+1]>CCD_Threshold &&ADV[i+2]>CCD_Threshold &&ADV[i+3]<CCD_Threshold &&ADV[i+4]<CCD_Threshold &&ADV[i+5]<CCD_Threshold )
		 {	
			 Left=i+2;
			 break;	
		 }
	 }
	 for(j = 118;j>5; j--)//寻找右边跳变沿，连续三个黑像素后连续三个白像素判断右边跳变沿
   {
		if(ADV[j]<CCD_Threshold &&ADV[j+1]<CCD_Threshold &&ADV[j+2]<CCD_Threshold &&ADV[j+3]>CCD_Threshold &&ADV[j+4]>CCD_Threshold &&ADV[j+5]>CCD_Threshold )
		 {	
		   Right=j+2;
		   break;	
		 }
   }
	CCD_Median =(Right+Left)/2;//计算中线位置
	if(myabs(CCD_Median-Last_CCD_Median)>90)   //计算中线的偏差，如果太大
	CCD_Median=Last_CCD_Median;    //则取上一次的值
	Last_CCD_Median=CCD_Median;  //保存上一次的偏差
} 

/* USER CODE END 1 */
