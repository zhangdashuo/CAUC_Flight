/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file main.c
*@version V1.0
*@date  2018/5/24
*@brief 光流控制头文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#ifndef __OFPOSITION_CONTROL_H__
#define __OFPOSITION_CONTROL_H__

#include "stm32f4xx.h"

/* 实际输出的角度值 */
extern float expect_of_pitch_out,expect_of_roll_out;
/* 光流定点的速度环输出 */
void OF_SpeedControl_Mode(float Timer_t);


#endif
