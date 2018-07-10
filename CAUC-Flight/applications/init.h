/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file main.c
*@version V1.0
*@date  2018/5/24
*@brief 系统初始化头文件
*@design 无人机研究小组
	     谭政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#ifndef _INIT_H_
#define _INIT_H_

#include "stm32f4xx.h"

/* 初始化结束标识 */
extern u8 Init_Finish;

/*----------------------------------------------------------
 + 实现功能：飞控初始化
----------------------------------------------------------*/
extern void Light_Init(void);

#endif

