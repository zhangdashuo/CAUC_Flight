/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file main.c
*@version V1.0
*@date  2018/5/24
*@brief 气压计驱动头文件
*@design 无人机研究小组
	     谭政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#ifndef _DRIVER_MS5611_H
#define _DRIVER_MS5611_H

#include "stm32f4xx.h"
#include "device_iic.h"
#include "device_ms5611.h"
#include "driver_mpu6050.h"
#include "database.h"

/* 气压计硬件故障 */
extern u8 hard_error_ms5611;
/* 气压计计算高度，单位mm(毫米) */
extern int32_t baroAlt,baroAltOld;
/* 气压计计算速度单位mm/s */
extern float baro_alt_speed;

/*----------------------------------------------------------
 + 实现功能：气压计初始化
----------------------------------------------------------*/
extern void MS5611_Init(void);
/*----------------------------------------------------------
 + 实现功能：气压计数据更新 由任务调度调用周期10ms
----------------------------------------------------------*/
extern int MS5611_Update(void);
/*----------------------------------------------------------
 + 实现功能：气压计高度计算
----------------------------------------------------------*/
extern void MS5611_BaroAltCalculate(void);

#endif

