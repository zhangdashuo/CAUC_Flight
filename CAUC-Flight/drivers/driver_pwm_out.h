/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file main.c
*@version V1.0
*@date  2018/5/24
*@brief PWM输出设备头文件
*@design 无人机研究小组
	     谭政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#ifndef _DRIVER_PWM_OUT_H_
#define _DRIVER_PWM_OUT_H_

#include "stm32f4xx.h"
#include "device_pwm_out.h"

/* PWM的8个通道输出模式 */
extern int PWM_Mode;

/*----------------------------------------------------------
 + 实现功能：PWM的8个通道输出数据的调用
 + 调用参数功能：双字节整数数组：设置的数据(0-1000)
----------------------------------------------------------*/
extern void SetPwm(int16_t set_8pwm[]);

#endif

