/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file main.c
*@version V1.0
*@date  2018/5/24
*@brief 系统时间统计头文件
*@design 无人机研究小组
	     谭政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#ifndef _TIME_H_
#define _TIME_H_

#include "stm32f4xx.h"
#include "device_timer.h"

/*----------------------------------------------------------
 + 实现功能：延时
 + 调用参数：微秒
----------------------------------------------------------*/
extern void Delay_us(uint32_t);

/*----------------------------------------------------------
 + 实现功能：延时
 + 调用参数：毫秒
----------------------------------------------------------*/
extern void Delay_ms(uint32_t);

/*----------------------------------------------------------
 + 实现功能：计算两次点用时间间隔
 + 调用参数：统计时间项数组
 + 返回参数：两次时间间隔 单位：秒
----------------------------------------------------------*/
extern float Call_timer_cycle(u8);

/*----------------------------------------------------------
 + 实现功能：时间统计初始化
----------------------------------------------------------*/
extern void Cycle_Time_Init(void);

#endif

