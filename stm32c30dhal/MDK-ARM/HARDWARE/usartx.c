#include "usartx.h"
SEND_DATA Send_Data;
RECEIVE_DATA Receive_Data;
extern int Time_count;


u8 Usart1_Receive_buf[1];           //串口1接收中断数据存放的缓存区
u8 Usart1_Receive;                  //从串口1读取的数据
u8 Usart2_Receive_buf[1];          //串口2接收中断数据存放的缓冲区
u8 Usart2_Receive;                 //从串口2读取的数据
u8 Uart5_Receive_buf[1];          //串口5接收中断数据存放的缓冲区
u8 Uart5_Receive;                 //从串口5读取的数据
/**************************************************************************
Function: Refresh the OLED screen
Input   : none
Output  : none
函数功能：串口2接收中断
入口参数：无
返回  值：无
**************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{	
	if(huart->Instance==USART2)//如果是串口1
	{   
    static int temp_count = 0;				//用于记录前进的指令的次数，第一次连接蓝牙的时候需要用到
	  int Usart_Receive;		
		static u8 Flag_PID,i,j,Receive[50];
		static float Data;
				
		Usart_Receive=Usart2_Receive_buf[0];; //Read the data //读取数据

	  
		if(Usart_Receive==0x4B) 
			//Enter the APP steering control interface
		  //进入APP转向控制界面
			Turn_Flag=1;  
	  else	if(Usart_Receive==0x49||Usart_Receive==0x4A) 
      // Enter the APP direction control interface		
			//进入APP方向控制界面	
			Turn_Flag=0;	
		
		if(Turn_Flag==0) 
		{
			//App rocker control interface command
			//APP摇杆控制界面命令
			if(Usart_Receive>=0x41&&Usart_Receive<=0x48)  
			{	
				Flag_Direction=Usart_Receive-0x40;
			}
			else	if(Usart_Receive<=8)   
			{			
				Flag_Direction=Usart_Receive;
			}	
			else  Flag_Direction=0;
		}
		else if(Turn_Flag==1)
		{
			//APP steering control interface command
			//APP转向控制界面命令
			if     (Usart_Receive==0x43) Flag_Left=0,Flag_Right=1; //Right rotation //右自转
			else if(Usart_Receive==0x47) Flag_Left=1,Flag_Right=0; //Left rotation  //左自转
			else                         Flag_Left=0,Flag_Right=0;
			if     (Usart_Receive==0x41||Usart_Receive==0x45) 
			{
				if((++temp_count) == 5)					//需要连续发送5次前进的指令，上拉转盘一段时间可开始app控制
				{
					temp_count = 0;
					APP_ON_Flag = RC_ON;		
					PS2_ON_Flag = RC_OFF;
				}
				Flag_Direction=Usart_Receive-0x40;
			}
			else  Flag_Direction=0;
		}
		if(Usart_Receive==0x58)  RC_Velocity=RC_Velocity+100; //Accelerate the keys, +100mm/s //加速按键，+100mm/s
		if(Usart_Receive==0x59)  RC_Velocity=RC_Velocity-100; //Slow down buttons,   -100mm/s //减速按键，-100mm/s
	  
	 // The following is the communication with the APP debugging interface
	 //以下是与APP调试界面通讯
	 if(Usart_Receive==0x7B) Flag_PID=1;   //The start bit of the APP parameter instruction //APP参数指令起始位
	 if(Usart_Receive==0x7D) Flag_PID=2;   //The APP parameter instruction stops the bit    //APP参数指令停止位

	 if(Flag_PID==1) //Collect data //采集数据
	 {
		Receive[i]=Usart_Receive;
		i++;
	 }
	 if(Flag_PID==2) //Analyze the data //分析数据
	 {
			if(Receive[3]==0x50) 	 PID_Send=1;
			else  if(Receive[1]!=0x23) 
      {								
				for(j=i;j>=4;j--)
				{
					Data+=(Receive[j-1]-48)*pow(10,i-j);
				}
				
				if(Mode == APP_Control_Mode)
					{
					switch(Receive[1])			//APP电机调参
					 {
						 case 0x30:  RC_Velocity=Data;break;
						 case 0x31:  Velocity_KP=Data;break;
						 case 0x32:  Velocity_KI=Data;break;
						 case 0x33:  break;
						 case 0x34:  break;
						 case 0x35:  break;
						 case 0x36:  break;
						 case 0x37:  break;
						 case 0x38:  break; 				
					 } 
					}
					else if(Mode == ELE_Line_Patrol_Mode)		//电磁巡线z轴速度调参
					{
					switch(Receive[1])
					 {
						 case 0x30:  RC_Velocity_ELE=Data;break;
						 case 0x31:  ELE_KP=Data;break;
						 case 0x32:  ELE_KI=Data;break;
						 case 0x33:  break;
						 case 0x34:  break;
						 case 0x35:  break;
						 case 0x36:  break;
						 case 0x37:  break;
						 case 0x38:  break; 				
					 } 
					}
					else 	if(Mode == CCD_Line_Patrol_Mode)									//CCD巡线z轴速度调参
					{
					switch(Receive[1])
					 {
						 case 0x30:  RC_Velocity_CCD=Data;break;
						 case 0x31:  CCD_KP=Data;break;
						 case 0x32:  CCD_KI=Data;break;
						 case 0x33:  break;
						 case 0x34:  break;
						 case 0x35:  break;
						 case 0x36:  break;
						 case 0x37:  break;
						 case 0x38:  break; 				
					 } 
					}
					else if(Mode == Lidar_Along_Mode)
					{
						switch(Receive[1])
						{
						 case 0x30: Akm_Along_Distance_KP=Data;break;
						 case 0x31:  Akm_Along_Distance_KD=Data;break;
						 case 0x32:  Akm_Along_Distance_KI=Data;break;
						 case 0x33:  break;
						 case 0x34:  break;
						 case 0x35:  break;
						 case 0x36:  break;
						 case 0x37:  break;
						 case 0x38:  break; 	
						}
					}
					else if(Mode == Lidar_Follow_Mode)
					{
						switch(Receive[1])
						{
						 case 0x30:  break;
						 case 0x31:  break;
						 case 0x32:  break;
						 case 0x33:  break;
						 case 0x34:  break;
						 case 0x35:  break;
						 case 0x36:  break;
						 case 0x37:  break;
						 case 0x38:  break; 	
						}
					}
				}		
      //Relevant flag position is cleared			
      //相关标志位清零			
			Flag_PID=0;
			i=0;
			j=0;
			Data=0;
			memset(Receive, 0, sizeof(u8)*50); //Clear the array to zero//数组清零
	 }
   if(RC_Velocity<0)   RC_Velocity=0; 
	 if(RC_Velocity_CCD<0)   RC_Velocity_CCD=0;
	 if(RC_Velocity_ELE<0)   RC_Velocity_ELE=0;		 
	 HAL_UART_Receive_IT(&huart2,Usart2_Receive_buf,sizeof(Usart2_Receive_buf));//串口3回调函数执行完毕之后，需要再次开启接收中断等待下一次接收中断的发生
  }
  else if(huart->Instance==UART5)
	{
		static u8 state = 0;//状态位	
		static u8 crc_sum = 0;//校验和
		static u8 cnt = 0;//用于一帧16个点的计数
		u8 temp_data;
		temp_data=Uart5_Receive_buf[0];;	
		switch(state)
		{
			case 0:
				if(temp_data == HEADER_0)//头固定
				{
					Pack_Data.header_0= temp_data;
					state++;
					//校验
					crc_sum += temp_data;
				} else state = 0,crc_sum = 0;
				break;
			case 1:
				if(temp_data == HEADER_1)//头固定
				{
					Pack_Data.header_1 = temp_data;
					state++;
					crc_sum += temp_data;
				} else state = 0,crc_sum = 0;
				break;
			case 2:
				if(temp_data == Length_)//字长固定
				{
					Pack_Data.ver_len = temp_data;
					state++;
					crc_sum += temp_data;
				} else state = 0,crc_sum = 0;
				break;
			case 3:
				Pack_Data.speed_h = temp_data;//速度高八位
				state++;
				crc_sum += temp_data;			
				break;
			case 4:
				Pack_Data.speed_l = temp_data;//速度低八位
				state++;
				crc_sum += temp_data;
				break;
			case 5:
				Pack_Data.start_angle_h = temp_data;//开始角度高八位
				state++;
				crc_sum += temp_data;
				break;
			case 6:
				Pack_Data.start_angle_l = temp_data;//开始角度低八位
				state++;
				crc_sum += temp_data;
				break;
			
			case 7:case 10:case 13:case 16:
			case 19:case 22:case 25:case 28:
			case 31:case 34:case 37:case 40:
			case 43:case 46:case 49:case 52:
				
			case 55:case 58:case 61:case 64:
			case 67:case 70:case 73:case 76:
			case 79:case 82:case 85:case 88:
			case 91:case 94:case 97:case 100:
				Pack_Data.point[cnt].distance_h = temp_data & 0x7f ;//16个点的距离数据，高字节
				state++;
				crc_sum += temp_data;
				break;
			
			case 8:case 11:case 14:case 17:
			case 20:case 23:case 26:case 29:
			case 32:case 35:case 38:case 41:
			case 44:case 47:case 50:case 53:
				
			case 56:case 59:case 62:case 65:
			case 68:case 71:case 74:case 77:
			case 80:case 83:case 86:case 89:
			case 92:case 95:case 98:case 101:
				Pack_Data.point[cnt].distance_l = temp_data;//16个点的距离数据，低字节
				state++;
				crc_sum += temp_data;
				break;
			
			case 9:case 12:case 15:case 18:
			case 21:case 24:case 27:case 30:
			case 33:case 36:case 39:case 42:
			case 45:case 48:case 51:case 54:
				
			case 57:case 60:case 63:case 66:
			case 69:case 72:case 75:case 78:
			case 81:case 84:case 87:case 90:
			case 93:case 96:case 99:case 102:
				Pack_Data.point[cnt].Strong = temp_data;//16个点的强度数据
				state++;
				crc_sum += temp_data;
				cnt++;
				break;
			case 103:case 104:
				state++;
				crc_sum += temp_data;
				cnt++;
				break;
			case 105:
				Pack_Data.end_angle_h = temp_data;//结束角度的高八位
				state++;
				crc_sum += temp_data;			
				break;
			case 106:
				Pack_Data.end_angle_l = temp_data;//结束角度的低八位
				state++;
				crc_sum += temp_data;
				break;
			case 107:
				Pack_Data.crc = temp_data;//校验
				state = 0;
				cnt = 0;
				if(crc_sum == Pack_Data.crc)
				{
					data_process();//数据处理，校验正确不断刷新存储的数据	
				}
				else 
				{
					memset(&Pack_Data,0,sizeof(Pack_Data));//清零
				}
				crc_sum = 0;//校验和清零
				break;
			default: break;
		}
		 HAL_UART_Receive_IT(&huart5,Uart5_Receive_buf,sizeof(Uart5_Receive_buf));//开启串口5接收中断
	}
	else if(huart->Instance==USART1)//串口一接收数据
	{
		Usart1_Receive = Usart1_Receive_buf[0];//读取数据
		HAL_UART_Receive_IT(&huart1,Usart1_Receive_buf,sizeof(Usart1_Receive_buf));//开启串口1接收中断
	}
}

/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type data, the unit reduction is converted
Input   : 8 bits high, 8 bits low
Output  : The target velocity of the robot on the X/Y/Z axis
函数功能：将上位机发过来目标前进速度Vx、目标角速度Vz，转换为阿克曼小车的右前轮转角
入口参数：目标前进速度Vx、目标角速度Vz，单位：m/s，rad/s
返回  值：阿克曼小车的右前轮转角，单位：rad
**************************************************************************/
float Vz_to_Akm_Angle(float Vx, float Vz)
{
	float R, AngleR, Min_Turn_Radius;
	//float AngleL;
	
	//Ackermann car needs to set minimum turning radius
	//If the target speed requires a turn radius less than the minimum turn radius,
	//This will greatly improve the friction force of the car, which will seriously affect the control effect
	//阿克曼小车需要设置最小转弯半径
	//如果目标速度要求的转弯半径小于最小转弯半径，
	//会导致小车运动摩擦力大大提高，严重影响控制效果
	Min_Turn_Radius=MINI_AKM_MIN_TURN_RADIUS;
	
	if(Vz!=0 && Vx!=0)
	{
		//If the target speed requires a turn radius less than the minimum turn radius
		//如果目标速度要求的转弯半径小于最小转弯半径
		if(float_abs(Vx/Vz)<=Min_Turn_Radius)
		{
			//Reduce the target angular velocity and increase the turning radius to the minimum turning radius in conjunction with the forward speed
			//降低目标角速度，配合前进速度，提高转弯半径到最小转弯半径
			if(Vz>0)
				Vz= float_abs(Vx)/(Min_Turn_Radius);
			else	
				Vz=-float_abs(Vx)/(Min_Turn_Radius);	
		}		
		R=Vx/Vz;
		//AngleL=atan(Axle_spacing/(R-0.5*Wheel_spacing));
		AngleR=atan(Axle_spacing/(R+0.5f*Wheel_spacing));
	}
	else
	{
		AngleR=0;
	}
	
	return AngleR;
}
/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type data, the unit reduction is converted
Input   : 8 bits high, 8 bits low
Output  : The target velocity of the robot on the X/Y/Z axis
函数功能：将上位机发过来的高8位和低8位数据整合成一个short型数据后，再做单位还原换算
入口参数：高8位，低8位
返回  值：机器人X/Y/Z轴的目标速度
**************************************************************************/
float XYZ_Target_Speed_transition(u8 High,u8 Low)
{
	//Data conversion intermediate variable
	//数据转换的中间变量
	short transition; 
	
	//将高8位和低8位整合成一个16位的short型数据
	//The high 8 and low 8 bits are integrated into a 16-bit short data
	transition=((High<<8)+Low); 
	return 
		transition/1000+(transition%1000)*0.001; //Unit conversion, mm/s->m/s //单位转换, mm/s->m/s						
}
/**************************************************************************
Function: Serial port 1 sends data
Input   : The data to send
Output  : none
函数功能：串口1发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart1_send(u8 data)
{
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	
}
/**************************************************************************
Function: Serial port 2 sends data
Input   : The data to send
Output  : none
函数功能：串口2发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart2_send(u8 data)
{
	USART2->DR = data;
	while((USART2->SR&0x40)==0);	
}


