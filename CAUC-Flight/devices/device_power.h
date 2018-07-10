/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file main.c
*@version V1.0
*@date  2018/5/24
*@brief 电池电压检测设备头文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#ifndef _DEVICE_POWER_H_
#define	_DEVICE_POWER_H_

#include "stm32f4xx.h"

/* POWER 的GPIO宏定义 */
#define RCC_POWER				RCC_AHB1Periph_GPIOA
#define RCC_ADC					RCC_APB2Periph_ADC1
#define GPIO_POWER			GPIOA
#define Pin_POWER				GPIO_Pin_1

/*----------------------------------------------------------
 + 实现功能：控制POWER初始化
----------------------------------------------------------*/
extern void POWER_Init(void);

/*----------------------------------------------------------
 + 实现功能：采集电池电压
----------------------------------------------------------*/
extern uint16_t Call_POWER_show(void);

#endif
