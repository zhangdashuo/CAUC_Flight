/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file device_timer.c
*@version V1.0
*@date  2018/5/24
*@brief 定时器设备文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#include "stm32f4xx.h"
#include "device_timer.h"

/*----------------------------------------------------------
 + 实现功能：开启定时器
----------------------------------------------------------*/
void SysTick_Configuration(void)
{
    /* RCC时钟频率 */
    RCC_ClocksTypeDef  rcc_clocks;
    /* SysTick两次中断期间计数次数 */
    uint32_t         cnts;
    /* 获取RCC时钟频率 */
    RCC_GetClocksFreq(&rcc_clocks);
    /* 由时钟源及分频系数计算计数次数 */
    cnts = (uint32_t)rcc_clocks.HCLK_Frequency / TICK_PER_SECOND;
    cnts = cnts / 8;
    /* 设置SysTick计数次数 */
    SysTick_Config(cnts);
    /* 配置SysTick时钟源 */
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
}

