/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file main.c
*@version V1.0
*@date  2018/5/24
*@brief 蜂鸣器指示灯状态控制头文件
*@design 无人机研究小组
	     谭政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#ifndef _DRIVER_BEEP_H_
#define	_DRIVER_BEEP_H_

#include "stm32f4xx.h"
#include "device_led.h"

/* 每个BEEP范围是0-20 */
extern u8 BEEP_Brightness;

/*----------------------------------------------------------
 + 实现功能：由任务调度调用周期50ms
----------------------------------------------------------*/
extern void Call_BEEP_duty(void);

/*----------------------------------------------------------
 + 实现功能：MPU故障指示
----------------------------------------------------------*/
extern void BEEP_MPU_Err(void);

/*----------------------------------------------------------
 + 实现功能：磁力计故障指示
----------------------------------------------------------*/
extern void BEEP_Mag_Err(void);

/*----------------------------------------------------------
 + 实现功能：气压计故障指示
----------------------------------------------------------*/
extern void BEEP_MS5611_Err(void);

#endif

