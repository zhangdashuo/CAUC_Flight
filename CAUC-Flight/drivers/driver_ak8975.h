/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file main.c
*@version V1.0
*@date  2018/5/24
*@brief 磁力计（电子罗盘）驱动头文件
*@design 无人机研究小组
	     谭政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#ifndef _DRIVER_AK8975_H_
#define	_DRIVER_AK8975_H_

#include "stm32f4xx.h"
#include "database.h"

/* 磁力计数组：采样值,偏移值,纠正后的值 */
extern ak8975_t ak8975;
/* 磁力计硬件故障 */
extern u8 hard_error_ak8975;
/* 磁力计校准标识 */
extern u8 Mag_CALIBRATED;

/*----------------------------------------------------------
 + 实现功能：磁力计采样触发
 + 返回值：磁力计运行状态
----------------------------------------------------------*/
extern uint8_t AK8975_IS_EXIST(void);

/*----------------------------------------------------------
 + 实现功能：由任务调度调用周期10ms
----------------------------------------------------------*/
extern void Call_AK8975(void);

#endif

