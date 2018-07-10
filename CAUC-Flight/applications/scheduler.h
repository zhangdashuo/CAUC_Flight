/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file main.c
*@version V1.0
*@date  2018/5/24
*@brief 任务调度头文件
*@design 无人机研究小组
	     谭政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_

#include "stm32f4xx.h"

/* 循环计数结构体 */
typedef struct
{
    /* 循环运行完毕标志 */
    u8 check_flag;
    /* 代码在预定周期内没有运行完错误计数 */
    u8 err_flag;
    /* 不同周期的执行任务独立计时 */
    s16 cnt_2ms;
    s16 cnt_5ms;
    s16 cnt_10ms;
    s16 cnt_20ms;
    s16 cnt_50ms;
    s16 cnt_100ms;
} loop_t;

/*----------------------------------------------------------
 + 实现功能：主循环 由主函数调用
----------------------------------------------------------*/
extern void Main_Loop(void);

/*----------------------------------------------------------
 + 实现功能：由Systick定时器调用 周期：1毫秒
----------------------------------------------------------*/
extern void Call_Loop_timer(void);

#endif

