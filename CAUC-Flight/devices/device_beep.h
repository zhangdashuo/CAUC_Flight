/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file main.c
*@version V1.0
*@date  2018/5/24
*@brief 蜂鸣器设备头文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#ifndef _DEVICE_BEEP_H_
#define	_DEVICE_BEEP_H_

#include "stm32f4xx.h"

/* BEEP 电平宏定义 */
#define BEEP_ON         	GPIO_BEEP->BSRRL = Pin_BEEP
#define BEEP_OFF          GPIO_BEEP->BSRRH = Pin_BEEP

/* BEEP 的GPIO宏定义 */
#define RCC_BEEP			RCC_AHB1Periph_GPIOA
#define GPIO_BEEP			GPIOA
#define Pin_BEEP			GPIO_Pin_0

/*----------------------------------------------------------
 + 实现功能：控制BEEP初始化
----------------------------------------------------------*/
extern void BEEP_Init(void);

/*----------------------------------------------------------
 + 实现功能：控制BEEP 由任务调度调用周期1ms
----------------------------------------------------------*/
extern void Call_BEEP_show(u8 duty);

#endif
