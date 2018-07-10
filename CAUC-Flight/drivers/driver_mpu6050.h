/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file main.c
*@version V1.0
*@date  2018/5/24
*@brief IMU传感器mpu6050驱动头文件
*@design 无人机研究小组
	     谭政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#ifndef _DRIVER_MPU6050_H
#define _DRIVER_MPU6050_H

#include "stm32f4xx.h"
#include "database.h"
#include "device_mpu6050.h"

/* MPU6050结构体 */
extern MPU6050_STRUCT mpu6050;

/*----------------------------------------------------------
 + 实现功能：由任务调度调用周期2ms
----------------------------------------------------------*/
extern void Call_MPU6050_Data_Prepare(float T);

#endif

