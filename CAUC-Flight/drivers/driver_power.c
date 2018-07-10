/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file driver_power.c
*@version V1.0
*@date  2018/5/24
*@brief 电池电压检测控制文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#include "stm32f4xx.h"
#include "driver_power.h"
#include "device_power.h"

/* 滑动窗口滤波数值个数 */
#define POWER_FILTER_NUM 					10

/* 滤波滑动窗口数组 */
float POWER_FILT_BUF[(POWER_FILTER_NUM + 1)];

/* 滤波滑动窗口数组下标 */
uint8_t power_filter_cnt = 0,power_filter_cnt_old = 0;
/* 电池电压 */
uint16_t Electric_quantity = 0;
/* 低电压模式 */
uint8_t LowPower_Flag = 0;


void Call_POWER_duty()
{
	float temp = 0;
	float POWER_FILT_TMP = 0;
	/* 连续低电压计数 */
	static uint8_t lowpowercnt = 0;
	
	temp = Call_POWER_show();
	temp = temp * ( 1260.0f / (12.6f/23.3f*4096) );
	
	 /* 滑动窗口滤波数组下标移位 */
	if( ++power_filter_cnt > POWER_FILTER_NUM )
	{
		power_filter_cnt = 0;
		power_filter_cnt_old = 1;
	}
	else
		power_filter_cnt_old = (power_filter_cnt == POWER_FILTER_NUM)? 0 : (power_filter_cnt + 1);
	
	/* 更新滤波滑动窗口临时数组 */
	POWER_FILT_BUF[power_filter_cnt] = temp;
	
	/* 更新滤波滑动窗口数组 */
	for(u8 i=0; i<POWER_FILTER_NUM; i++)
	{
		POWER_FILT_TMP += POWER_FILT_BUF[i];
	}
	
	/* 得出处理后的数据 */
	Electric_quantity  = (uint16_t)((float)( POWER_FILT_TMP )/(float)POWER_FILTER_NUM);	
	
	/* 判断是否为连续低电压状态 */
	if( Electric_quantity < 1100)
	{
		if( lowpowercnt < 250)
			lowpowercnt ++;
	}
	else
		lowpowercnt = 0;
	
	/* 低电压状态指示 */
	if( 250 == lowpowercnt )
		LowPower_Flag = 1;
	else
		LowPower_Flag = 0;
	
}
