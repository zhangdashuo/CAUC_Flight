/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file time.c
*@version V1.0
*@date  2018/5/24
*@brief 系统时间统计文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#include "stm32f4xx.h"
#include "time.h"
#include "init.h"
#include "scheduler.h"

/* 设置获取时间的数组数量 */
#define GET_TIME_NUM 	    (3)

/* 换算成SYStick定时器计数微秒单位的比例 */
#define TICK_US	(1000000/TICK_PER_SECOND)

/* 复位起的总中断次数 */
volatile uint32_t sysTickUptime = 0;

/* 计算两次点用时间间隔的数组 */
volatile float Cycle_T[GET_TIME_NUM][3];

/* 数组下标枚举体 */
enum
{
    NOW = 0,
    OLD,
    NEW,
};

/*----------------------------------------------------------
 + 实现功能：获取系统总运行时间
 + 返回参数：微秒
----------------------------------------------------------*/
uint32_t GetSysTime_us(void)
{
    /* 系统总运行 单位：毫秒 */
    register uint32_t ms;
    /* 用于返回的变量 单位：微秒 */
    u32 value;
    /* 获取系统总运行 单位：毫秒 */
    ms = sysTickUptime;
    /* 计算并获取为系统总运行 单位：微秒 */
    value = ms * TICK_US + (SysTick->LOAD - SysTick->VAL) * TICK_US / SysTick->LOAD;
    /* 返回参数 单位：微秒 */
    return value;
}

/*----------------------------------------------------------
 + 实现功能：延时
 + 调用参数：微秒
----------------------------------------------------------*/
void Delay_us(uint32_t us)
{
    /* 获取系统时钟 单位微秒 */
    uint32_t now = GetSysTime_us();
    /* 比较两次时间差 */
    while (GetSysTime_us() - now < us);
}

/*----------------------------------------------------------
 + 实现功能：延时
 + 调用参数：毫秒
----------------------------------------------------------*/
void Delay_ms(uint32_t ms)
{
    /* 延时微秒次数 */
    while (ms--) Delay_us(1000);
}

/*----------------------------------------------------------
 + 实现功能：计算两次点用时间间隔
 + 调用参数：统计时间项数组
 + 返回参数：两次时间间隔 单位：秒
----------------------------------------------------------*/
float Call_timer_cycle(u8 item)
{
    /* 上一次的时间 */
    Cycle_T[item][OLD] = Cycle_T[item][NOW];
    /* 本次的时间 */
    Cycle_T[item][NOW] = GetSysTime_us()/1000000.0f;
    /* 间隔的时间 单位：秒 */
    Cycle_T[item][NEW] = ( ( Cycle_T[item][NOW] - Cycle_T[item][OLD] ) );
    /* 两次时间间隔 单位：秒 */
    return Cycle_T[item][NEW];
}

/*----------------------------------------------------------
 + 实现功能：时间统计初始化
----------------------------------------------------------*/
void Cycle_Time_Init()
{
    /* 计算两次点用时间间隔数组清零 */
    for(u8 i=0; i<GET_TIME_NUM; i++)
        Call_timer_cycle(i);
}

/*----------------------------------------------------------
 + 实现功能：由SysTick定时中断调用，周期1ms
----------------------------------------------------------*/
void SysTick_Handler(void)
{
    /* 复位起的总中断次数 */
    sysTickUptime++;
    /* 系统没有初始化好，不进行大循环 */
    if( ! Init_Finish) return;
    /* 大循环统计 */
    Call_Loop_timer();
}

