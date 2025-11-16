#ifndef __BALANCE_H
#define __BALANCE_H			  	 
#include "sys.h"
#include "stdio.h"
#include "system.h"
#include "lidar.h"
#include "main.h"

#define Follow_Distance 1600  //跟随距离
#define Keep_Follow_Distance 500   //跟随保持距离
#define Avoid_Min_Distance 200  //避障最小距离
#define Avoid_Distance 450     //避障距离

#define forward_velocity 0.3  //Move_X速度
#define corner_velocity 0.5    //Move_Y速度
#define other_corner_velocity 1.5    //Move_Y速度

#define amplitude_limiting 0.6   //速度限幅


//Parameter of kinematics analysis of omnidirectional trolley
//全向轮小车运动学分析参数
#define X_PARAMETER    (sqrt(3)/2.f)               
#define Y_PARAMETER    (0.5f)    
#define L_PARAMETER    (1.0f)

extern u8 Lidar_Detect;
extern u8 Mode;

#define Lidar_Detect_ON						1				//电磁巡线是否开启雷达检测障碍物
#define Lidar_Detect_OFF					0
#define Barrier_Detected						1
#define No_Barrier								0

//小车各模式定义
#define PS2_Control_Mode					1
#define APP_Control_Mode          0
#define Lidar_Avoid_Mode					2
#define Lidar_Follow_Mode					3
#define ELE_Line_Patrol_Mode			5
#define CCD_Line_Patrol_Mode			6
#define Lidar_Along_Mode				  4			


//指示遥控控制的开关
#define RC_ON								1	
#define RC_OFF								0
//遥控控制前后速度最大值
#define MAX_RC_Velocity						1.230f
//遥控控制转向速度最大值
#define	MAX_RC_Turn_Bias					2.27f
//遥控控制前后速度最小值
#define MINI_RC_Velocity					0.1f
//遥控控制转向速度最小值
#define	MINI_RC_Turn_Velocity				0.2f
//前进加减速幅度值，每次遥控加减的步进值
#define X_Step								0.08f
//转弯加减速幅度值
#define Z_Step								0.2f


extern float Mec_Car_x1,Mec_Car_x2;
extern float Omni_Car_x2,Omni_Car_x1;
extern float RC_Velocity_CCD, RC_Velocity_ELE; 
extern float PS2_Velocity,PS2_Turn_Velocity;
extern int Time_count;

void delay1_us(u16 time);
void delay1_ms(u16 time);
void  Get_RC_CCD(void);
void  Get_RC_ELE(void);
void Balance_task(void *pvParameters);
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo);
void Limit_Pwm(int amplitude);
float target_limit_float(float insert,float low,float high);
int target_limit_int(int insert,int low,int high);
u8 Turn_Off( int voltage);
u32 myabs(long int a);
int Incremental_PI_A (float Encoder,float Target);
int Incremental_PI_B (float Encoder,float Target);
int Incremental_PI_C (float Encoder,float Target);
int Incremental_PI_D (float Encoder,float Target);
void Get_RC(void);
void Drive_Motor(float Vx,float Vy,float Vz);
void Get_Velocity_Form_Encoder(void);
void Smooth_control(float vx,float vy,float vz);
float float_abs(float insert);
void PS2_Control(void);
void ELE_Move_Detect(void);
u8 Detect_Barrier(void);
void Lidar_Avoid(void);
void Lidar_Follow(void);
void Lidar_along_wall(void);
#endif  

