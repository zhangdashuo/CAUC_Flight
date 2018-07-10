/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file main.c
*@version V1.0
*@date  2018/5/24
*@brief 超声波测距驱动头文件
*@design 无人机研究小组
	     谭政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#ifndef _DRIVER_ULTRASONIC_H
#define _DRIVER_ULTRASONIC_H

#include "stm32f4xx.h"
#include "database.h"

/* 超声波模块状态标志位：1准备接收高位 2准备接受低位 */
extern s8 ultra_state;
/* 超声波模块测量距离：单位毫米 */
extern u16 ultra_distance;
/* 超声波模块测量距离两次差：单位毫米 */
extern s16 ultra_delta;

/*----------------------------------------------------------
 + 实现功能：超声波模块初始化
----------------------------------------------------------*/
void Ultrasonic_Init(void);

/*----------------------------------------------------------
 + 实现功能：由任务调度调用周期100ms
----------------------------------------------------------*/
void Call_Ultrasonic(void);

/*----------------------------------------------------------
 + 实现功能：串口5接收到数据解析
 + 调用参数功能：单字节整数：串口接收到的数据
----------------------------------------------------------*/
void Ultra_Get(u8 com_data);

#endif

