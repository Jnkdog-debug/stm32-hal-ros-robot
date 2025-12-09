#include "ackermann_chassis.h"
#include "math.h"
#include <stdio.h>


#define PI 3.141592654f

/**
 * @brief 线速度转换为车轮转速（转/秒）
 * @param self 阿克曼底盘对象指针
 * @param speed 线速度（mm/s）
 * @return 车轮转速（转/秒），计算公式：speed / (π * 轮径)
 */
static inline float linear_speed_to_rps(AckermannChassisTypeDef *self,  float speed)
{
    return speed / (PI * self->wheel_diameter);
}

/*vx mm/s
  angule_rate rad/s
*/

/**
 * @brief 根据转向角度计算右侧车轮速度
 * @details 阿克曼转向底盘中，前轮转向时两侧车轮需要不同速度以实现转向运动
 *          该函数通过转向舵机位置计算需要的速度调整比例
 * @param self 阿克曼底盘对象指针
 * @param size 舵机PWM信号大小（500为直行，500-625为右转，375-500为左转）
 * @param v1 左侧车轮速度（转/秒）
 * @return 右侧车轮速度（转/秒）
 */
float ackermann_velocity_difference(AckermannChassisTypeDef *self,int size,float v1){
	int angle = 0;
	if (size > 500){angle = (size-500)/(1000/240);}
	else if(size == 500){float v2=v1;return v2;}
	else {angle = (500-size)/(1000/240);}
	
	double t = tan((angle*PI)/180);
	float v_t = ((AckermannChassisTypeDef*)self)->shaft_length / t + (((AckermannChassisTypeDef*)self)->wheel_diameter/2);
	float v2 = v1*((v_t+(((AckermannChassisTypeDef*)self)->wheel_diameter/2))/(v_t-(((AckermannChassisTypeDef*)self)->wheel_diameter/2)));
	
	return v2;
}

/**
 * @brief 阿克曼底盘运动控制函数
 * @details 根据线速度和转向半径计算左右车轮速度和舵机转角，实现底盘运动
 *          支持前进、后退和转向等各种运动方式
 * @param self 阿克曼底盘对象指针
 * @param vx 底盘线速度（mm/s），正值前进，负值后退
 * @param r 转向半径（mm），正值右转，负值左转，0表示直行
 * @return 无返回值，通过调用set_motors函数设置电机速度和舵机角度
 */
void ackermann_chassis_move(AckermannChassisTypeDef *self, float vx, float r )
{   
	  int servos = 500;
	  float rps_r ;
	  bool swerve = true;
	  float steering_ratio; 
	  float rps_l = linear_speed_to_rps(self, vx);
	  if(r == 0){steering_ratio = 0;}else{steering_ratio = ((AckermannChassisTypeDef*)self)->shaft_length / fabs(r);}
//		printf("%f/n",steering_ratio);
		if (r>0){swerve = true;}else if(r<0){swerve = false;}
		if (steering_ratio > tan(PI/180*30)){steering_ratio = tan(PI/180*30);}
		if (swerve == true){steering_ratio = steering_ratio;}else{steering_ratio = -steering_ratio;}
	  if(rps_l !=0){
		    if(steering_ratio != 0){
					  servos = (steering_ratio * 187.5)+500;
					  
					  if (servos > 625){servos=625;}else if(servos < 375){servos=375;}
				}else if(steering_ratio == 0){
				    servos = 500;
				}
		
		rps_r = ackermann_velocity_difference(self,servos,rps_l);		
		}else{rps_r = rps_l;}
//		printf("%d/n",servos);
		if (swerve == true){self->set_motors(self, -rps_r, rps_l,servos);;
		}else{self->set_motors(self, rps_l,-rps_r,servos);}
    
}

/**
 * @brief 停止底盘运动
 * @details 立即停止所有电机和舵机，使底盘处于静止状态
 * @param self 底盘对象指针（通过void指针传递）
 * @return 无返回值
 */
static void stop(void *self)
{
    ((AckermannChassisTypeDef*)self)->set_motors(self, 0, 0,500);
}

/**
 * @brief 通过速度和转向半径设置底盘运动（标准接口函数）
 * @details 这是通用接口函数，用于与底盘抽象层兼容
 *          转向半径优先级高于vy参数，vy参数在本实现中未使用
 * @param self 底盘对象指针（通过void指针传递）
 * @param vx 底盘线速度（mm/s）
 * @param vy 保留参数，本实现不使用
 * @param r 转向半径（mm），正值右转，负值左转
 * @return 无返回值
 */
static void set_velocity(void *self, float vx, float vy, float r)
{
    ackermann_chassis_move(self, vx, r);
}

/**
 * @brief 通过线速度和转向半径设置底盘运动（半径模式接口）
 * @details 根据线速度方向和转向半径自动调整前进/后退方向，实现更智能的转向运动
 * @param self 底盘对象指针（通过void指针传递）
 * @param linear 底盘线速度大小（mm/s，绝对值）
 * @param r 转向半径（mm），可正可负表示转向方向
 * @param swerve 保留参数，本实现未使用
 * @return 无返回值
 */
static void set_velocity_radius(void* self, float linear, float r,bool swerve)
{
	if( 0 > r )
	{
		ackermann_chassis_move(self, -linear, r);
	} else {
		ackermann_chassis_move(self, linear, r);
	}
}

/**
 * @brief 阿克曼底盘对象初始化函数
 * @details 将底盘接口函数指针指向具体实现，建立虚函数表
 *          必须在使用底盘对象之前调用此函数
 * @param self 阿克曼底盘对象指针
 * @return 无返回值
 */
void ackermann_chassis_object_init(AckermannChassisTypeDef *self){
    self->base.stop = stop;
    self->base.set_velocity = set_velocity;
    self->base.set_velocity_radius = set_velocity_radius;
}


