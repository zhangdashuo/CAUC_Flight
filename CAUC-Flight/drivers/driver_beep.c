/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file driver_beep.c
*@version V1.0
*@date  2018/5/24
*@brief 蜂鸣器指示灯状态控制文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#include "stm32f4xx.h"
#include "driver_beep.h"
#include "device_beep.h"
#include "driver_power.h"
#include "ctrl.h"

/* BEEP范围是0-20 */
u8 BEEP_Brightness;

/*----------------------------------------------------------
 + 实现功能：由任务调度调用周期50ms
----------------------------------------------------------*/
void Call_BEEP_duty()
{
	/* 锁定时蜂鸣器鸣响时间 */
	static uint8_t lock_showtime = 0;
	/* 解锁时蜂鸣器鸣响时间 */
	static uint8_t unlock_showtime = 0 , unlock_showtime_1 = 0;
	

	/* 低电压状态指示 */
	if( LowPower_Flag )
	{
		BEEP_Brightness = 20;
	}
	/* 未解锁状态指示 */
	else if(!unlocked_to_fly)
	{
		lock_showtime = 0;
		if( unlock_showtime < 20)
		{
			unlock_showtime ++;
			/* 指示灯状态数组赋值 */
			BEEP_Brightness = 20;
		}
		else
			BEEP_Brightness = 0;
	}
	/* 解锁后状态指示 */
	else
	{
		unlock_showtime = 0;
		unlock_showtime_1++;
		if( lock_showtime < 40)
		{
			lock_showtime ++;
			/* 指示灯状态数组赋值 */
			BEEP_Brightness = 20;
		}
		else if( unlock_showtime_1 < 5 && Thr_Low)
		{
			BEEP_Brightness = 20;
		}
		else if( unlock_showtime_1 < 200)
		{
			BEEP_Brightness = 0;
		}
		else
			unlock_showtime_1 = 0;
	}
}
