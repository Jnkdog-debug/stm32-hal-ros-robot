#include "mecanum_chassis.h"
#include <math.h>
#include <stdio.h>
#include <stdbool.h>

#define PI 3.141592654f

/**
 * @brief 线速度转换为车轮转速（转/秒）
 * @details 将底盘线速度转换为车轮实际转速，考虑轮径和修正因子
 * @param self 麦克纳姆轮底盘对象指针
 * @param speed 线速度（mm/s）
 * @return 车轮转速（转/秒），计算公式：speed / (π * 轮径) * 修正因子
 */
static inline float linear_speed_to_rps(MecanumChassisTypeDef *self,  float speed)
{
    return speed / (PI * self->wheel_diameter) * self->correction_factor;
}

/**
 * @brief 使用极坐标（速度+方向）控制麦克纳姆轮底盘运动
 * @details 根据速度大小、运动方向和旋转角速度计算四个车轮速度
 *          麦克纳姆轮布局：
 *          v1 motor3 | ↑ x | motor1 v2
 *          +  y -    |     |
 *          v4 motor4 |     | motor2 v3
 * @param self 麦克纳姆轮底盘对象指针
 * @param speed 运动线速度（mm/s）
 * @param direction 运动方向角（0~2π），π/2向上↑，3π/2向右→
 * @param angular_rate 底盘旋转角速度（rad/s），正值逆时针旋转
 * @return 无返回值，通过set_motors函数设置四个电机速度
 */
void mecanum_chassis_set_velocity(MecanumChassisTypeDef *self, float speed, float direction, float angular_rate)
{
    /**
          Use polar coordinates to control moving
                      x
          v1 motor3|  ↑  |motor1 v2
            +  y - |     |
          v4 motor4|     |motor2 v3
          :param speed: mm/s
          :param direction: Moving direction 0~2pi, 1/2pi<--- ↑ ---> 3/2pi
          :param angular_rate:  The speed at which the chassis rotates rad/sec
          :param fake:
          :return:
          """
    */
    float vx = speed * sinf(direction);
    float vy = speed * cosf(direction);
    float vp = angular_rate * (self->wheelbase + self->shaft_length);
    float v1 = vy - vx - vp;
    float v2 = vy + vx + vp;
    float v3 = vy - vx + vp;
    float v4 = vy + vx - vp;
    v1 = linear_speed_to_rps(self, v1);
    v4 = linear_speed_to_rps(self, v4);
    v2 = linear_speed_to_rps(self, v2);
    v3 = linear_speed_to_rps(self, v3);
    self->set_motors(self, v1, v4, v2, v3);
}

/**
 * @brief 使用笛卡尔坐标（vx+vy）控制麦克纳姆轮底盘运动
 * @details 根据X轴速度、Y轴速度和旋转角速度直接计算四个车轮速度
 *          麦克纳姆轮布局：
 *          v1 motor3 | ↑ x | motor1 v2
 *          +  y -    |     |
 *          v4 motor4 |     | motor2 v3
 * @param self 麦克纳姆轮底盘对象指针
 * @param vx X轴速度（mm/s），正值向右
 * @param vy Y轴速度（mm/s），正值向上
 * @param angular_rate 底盘旋转角速度（rad/s），正值逆时针旋转
 * @return 无返回值，通过set_motors函数设置四个电机速度
 */
void mecanum_chassis_set_xy(MecanumChassisTypeDef *self, float vx, float vy, float angular_rate)
{
    /**
          Use polar coordinates to control moving
                      x
          v1 motor3|  ↑  |motor1 v2
            +  y - |     |
          v4 motor4|     |motor2 v3
          :param speed: mm/s
          :param direction: Moving direction 0~2pi, 1/2pi<--- ↑ ---> 3/2pi
          :param angular_rate:  The speed at which the chassis rotates rad/sec
          :param fake:
          :return:
          """
    */
    float vp = angular_rate * (self->wheelbase + self->shaft_length);
    float v1 = vx - vy - vp;
    float v2 = vx + vy + vp;
    float v3 = vx - vy + vp;
    float v4 = vx + vy - vp;
    v1 = linear_speed_to_rps(self, v1);
    v4 = linear_speed_to_rps(self, v4);
    v2 = linear_speed_to_rps(self, v2);
    v3 = linear_speed_to_rps(self, v3);
    self->set_motors(self, v1, v4, v2, v3);
}

/**
 * @brief 停止底盘运动
 * @details 立即停止所有四个电机，使底盘处于静止状态
 * @param self 底盘对象指针（通过void指针传递）
 * @return 无返回值
 */
static void stop(void *self)
{
    ((MecanumChassisTypeDef*)self)->set_motors(self, 0, 0, 0, 0);
}

/**
 * @brief 使用笛卡尔坐标设置底盘速度（标准接口函数）
 * @details 这是通用接口函数，将vx、vy和角速度传递给底盘控制函数
 * @param self 底盘对象指针（通过void指针传递）
 * @param vx X轴速度（mm/s），正值向右
 * @param vy Y轴速度（mm/s），正值向上
 * @param angular_rate 底盘旋转角速度（rad/s）
 * @return 无返回值
 */
static void set_velocity(void *self, float vx, float vy, float angular_rate)
{
    mecanum_chassis_set_xy(self, vx, vy, angular_rate);
}

/**
 * @brief 使用线速度和转向半径设置底盘运动（半径模式接口）
 * @details 支持两种转向模式：原地旋转和曲线运动
 *          麦克纳姆轮可以实现原地旋转和曲线转向等复杂运动
 * @param self 底盘对象指针（通过void指针传递）
 * @param linear 底盘线速度（mm/s）
 * @param r 转向半径（mm），用于计算角速度：angular_rate = linear / r
 * @param insitu 转向模式标志：true=原地旋转（不前进），false=曲线转向（同时前进）
 * @return 无返回值
 */
static void set_velocity_radius(void *self, float linear, float r, bool insitu)
{
	if(insitu) {
		float angular_rate = linear / r;
		mecanum_chassis_set_xy(self, 0, 0, angular_rate);
	}else{
		float angular_rate = linear / r;
		mecanum_chassis_set_xy(self, linear, 0, angular_rate);
	}
}

/**
 * @brief 麦克纳姆轮底盘对象初始化函数
 * @details 将底盘接口函数指针指向具体实现，建立虚函数表
 *          必须在使用底盘对象之前调用此函数
 * @param self 麦克纳姆轮底盘对象指针
 * @return 无返回值
 */
void mecanum_chassis_object_init(MecanumChassisTypeDef *self)
{
    self->base.stop = stop;
    self->base.set_velocity = set_velocity;
	self->base.set_velocity_radius = set_velocity_radius;
}
