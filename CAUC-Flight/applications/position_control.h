/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file main.c
*@version V1.0
*@date  2018/5/24
*@brief GPS位置控制头文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#ifndef __POSITION_CONTROL_H__
#define __POSITION_CONTROL_H__

#include "stm32f4xx.h"
#include "driver_GPS.h"

/* 悬停经纬度 坐标 单位 放大率10E5 */
extern int STOP_longitude,STOP_latitude;

/* 速度转换期望角度 */
extern float expect_angle_pitch,expect_angle_roll;

/* 当前位置 单位 放大率10E5 */
extern int t_longitude,t_latitude;

/* 方位 速度 放大率10E3 单位 毫米每秒 */
extern int speed_longitude,speed_latitude;

/* 飞行器 速度 单位毫米每秒 */
extern float Speed_Front,Speed_Left;
/* 飞行器位置向飞行器前方和左方偏移量 */
extern float Dist_Front,Dist_Left;
/* 期望速度的PID输出的角度控制量 */
extern float expect_speed_pitch,expect_speed_roll;
 /* 期望姿态的PID输出 */
extern float expect_angle_pitch,expect_angle_roll;
/* 期望速度的PID输出 */
extern float expect_speed_Front,expect_speed_Left;//

/*----------------------------------------------------------
 + 实现功能：串级PID控制悬停
 + 调用参数：两次调用时间间隔
----------------------------------------------------------*/
extern void PositionControl_Mode(float Timer_t);

#endif

