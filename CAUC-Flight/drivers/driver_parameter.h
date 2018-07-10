/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file main.c
*@version V1.0
*@date  2018/5/24
*@brief 默认参数及校准数据文件头文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#ifndef _DRIVER_PARAMETER_H
#define	_DRIVER_PARAMETER_H

#include "stm32f4xx.h"
#include "database.h"

/* PID参数数组 */
extern pid_setup_t pid_setup;

/*----------------------------------------------------------
 + 实现功能：PID参数恢复默认
----------------------------------------------------------*/
extern void Para_ResetToFactorySetup(void);

/*----------------------------------------------------------
 + 实现功能：所有参数初始化
----------------------------------------------------------*/
extern void Para_Init(void);

/*----------------------------------------------------------
 + 实现功能：保存加速度计的校准参数
----------------------------------------------------------*/
extern void Param_SaveAccelOffset(xyz_f_t *offset);

/*----------------------------------------------------------
 + 实现功能：保存陀螺仪的校准参数
----------------------------------------------------------*/
extern void Param_SaveGyroOffset(xyz_f_t *offset);

/*----------------------------------------------------------
 + 实现功能：保存磁力计的校准参数
----------------------------------------------------------*/
extern void Param_SaveMagOffset(xyz_f_t *offset);

/*----------------------------------------------------------
 + 实现功能：由任务调度调用周期20ms
----------------------------------------------------------*/
extern void Parameter_Save(void);

/*----------------------------------------------------------
 + 实现功能：所有参数初始化
----------------------------------------------------------*/
extern void PID_Para_Init(void);

#endif
