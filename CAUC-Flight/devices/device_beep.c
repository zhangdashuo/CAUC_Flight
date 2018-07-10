/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file device_beep.c
*@version V1.0
*@date  2018/5/24
*@brief 蜂鸣器设备文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#include "device_beep.h"
#include "stm32f4xx.h"

/*----------------------------------------------------------
 + 实现功能：控制BEEP初始化
----------------------------------------------------------*/
void BEEP_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_BEEP,ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//这里设置为开漏输出，因为要上拉到5V
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_Pin   = Pin_BEEP;
    GPIO_Init(GPIO_BEEP, &GPIO_InitStructure);

    GPIO_SetBits(GPIO_BEEP, Pin_BEEP);
}

/*----------------------------------------------------------
 + 实现功能：控制BEEP 由任务调度调用周期1ms
----------------------------------------------------------*/
void Call_BEEP_show( u8 duty )
{
    /* 计时值 */
    static u8 BEEP_cnt;

	/* 计时器用于比较 */
	if( BEEP_cnt < 19 )
		BEEP_cnt++;
	else
		BEEP_cnt = 0;

	/* 蜂鸣器开启状态控制 */
	if( BEEP_cnt < duty )
	{
		/* 蜂鸣器关闭 */
		BEEP_OFF;
	}
	/* 蜂鸣器关断状态控制 */
	else
	{
		/* 蜂鸣器打开 */
		BEEP_ON;
	}
}

