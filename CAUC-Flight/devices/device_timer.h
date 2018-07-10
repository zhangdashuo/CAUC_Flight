/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.
 + 文件名  ：device_timer.h
 + 描述    ：定时器设备头文件
代码尽量做到逐行注释 代码有问题，及时加群交流 作者有偿支持对开源代码的完善 */
#ifndef _DEVICE_TIMER_H_
#define _DEVICE_TIMER_H_

#include "stm32f4xx.h"

/* 每秒钟SYStick定时器计数值 */
#define TICK_PER_SECOND 1000

/*----------------------------------------------------------
 + 实现功能：开启定时器
----------------------------------------------------------*/
extern void SysTick_Configuration(void);
#endif
/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
