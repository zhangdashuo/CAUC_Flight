/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file main.c
*@version V1.0
*@date  2018/5/24
*@brief 主函数文件
*@design 无人机研究小组 上传至github进行共同研发
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#include "init.h"
#include "scheduler.h"

/*----------------------------------------------------------
 + 实现功能：主函数
----------------------------------------------------------*/
int32_t main(void)
{
    /* 飞控初始化 */
    Light_Init();
    /* 主循环 */
    while(1)Main_Loop();
}

