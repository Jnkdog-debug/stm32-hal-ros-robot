#include "led.h"

int Led_Count=500; //LED flicker time control //LED闪烁时间控制
/**************************************************************************
Function: LED interface initialization
Input   : none
Output  : none
函数功能：LED接口初始化
入口参数：无 
返回  值：无
**************************************************************************/

/**************************************************************************
Function: The LED flashing
Input   : none
Output  : blink time
函数功能：LED闪烁
入口参数：闪烁时间
返 回 值：无
**************************************************************************/
void Led_Flash(u16 time)
{
	  static int temp;
	  if(0==time) LED=1;
	  else		if(++temp==time)	LED=~LED,temp=0;
}

void Buzzer_Alarm(u16 count)
{
	int count_time;
	if(0 == count)
	{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);//置低电平，蜂鸣器不响
	}
	else if(++count_time >= count)
	{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);//置高电平，蜂鸣器响
		count_time = 0;	
	}
}


