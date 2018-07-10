/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file main.c
*@version V1.0
*@date  2018/5/24
*@brief 电池电压检测控制头文件
*@design 无人机研究小组
	     谭政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#ifndef _DRIVER_POWER_H_
#define	_DRIVER_POWER_H_

#include "stm32f4xx.h"
#include "device_led.h"

/* 电池电压 */
extern uint16_t Electric_quantity;
/* 低电压模式 */
extern uint8_t LowPower_Flag;

void Call_POWER_duty(void);

#endif

