/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file main.c
*@version V1.0
*@date  2018/5/24
*@brief 串口设备头文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#ifndef _DEVICE_USART_H
#define _DEVICE_USART_H
#include "stm32f4xx.h"

/*----------------------------------------------------------
 + 实现功能：串口1初始化
 + 调用参数功能：
 - u32 bound：波特率
 - u8 Priority：中断主优先级
 - u8 SubPriority：中断从优先级
 - FunctionalState TXenable：发送中断使能
 - FunctionalState RXenable：就收中断使能
----------------------------------------------------------*/
extern void Device_Usart1_ENABLE_Init(u32 bound,u8 Priority,u8 SubPriority,FunctionalState TXenable,FunctionalState RXenable);

/*----------------------------------------------------------
 + 实现功能：串口2初始化
 + 调用参数功能：
 - u32 bound：波特率
 - u8 Priority：中断主优先级
 - u8 SubPriority：中断从优先级
 - FunctionalState TXenable：发送中断使能
 - FunctionalState RXenable：就收中断使能
----------------------------------------------------------*/
extern void Device_Usart2_ENABLE_Init(u32 bound,u8 Priority,u8 SubPriority,FunctionalState TXenable,FunctionalState RXenable);

/*----------------------------------------------------------
 + 实现功能：串口3初始化
 + 调用参数功能：
 - u32 bound：波特率
 - u8 Priority：中断主优先级
 - u8 SubPriority：中断从优先级
 - FunctionalState TXenable：发送中断使能
 - FunctionalState RXenable：就收中断使能
----------------------------------------------------------*/
extern void Device_Usart3_ENABLE_Init(u32 bound,u8 Priority,u8 SubPriority,FunctionalState TXenable,FunctionalState RXenable);

/*----------------------------------------------------------
 + 实现功能：串口4初始化
 + 调用参数功能：
 - u32 bound：波特率
 - u8 Priority：中断主优先级
 - u8 SubPriority：中断从优先级
 - FunctionalState TXenable：发送中断使能
 - FunctionalState RXenable：就收中断使能
----------------------------------------------------------*/
extern void Device_Usart4_ENABLE_Init(u32 bound,u8 Priority,u8 SubPriority,FunctionalState TXenable,FunctionalState RXenable);

/*----------------------------------------------------------
 + 实现功能：串口5初始化
 + 调用参数功能：
 - u32 bound：波特率
 - u8 Priority：中断主优先级
 - u8 SubPriority：中断从优先级
 - FunctionalState TXenable：发送中断使能
 - FunctionalState RXenable：就收中断使能
----------------------------------------------------------*/
extern void Device_Usart5_ENABLE_Init(u32 bound,u8 Priority,u8 SubPriority,FunctionalState TXenable,FunctionalState RXenable);

#endif
