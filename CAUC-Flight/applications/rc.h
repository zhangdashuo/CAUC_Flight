/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.
 + 文件名  ：rc.h
 + 描述    ：遥控器通道数据处理头文件
代码尽量做到逐行注释 代码有问题，及时加群交流 作者有偿支持对开源代码的完善 */
#ifndef _RC_H
#define	_RC_H

#include "stm32f4xx.h"

/* 滤波后的信号数据，范围是-500到+500 */
extern float CH_filter[];
/* 控制信号数据，范围是-500到+500 */
extern s16 CH[];
/* 8通道输入数传控制数据的数组1000-2000的控制信号 */
extern u16 RX_CH[];


/* 解锁判断标志
0未解锁 1已经解锁 */
extern u8 unlocked_to_fly;
/* 定高模式切换
0自稳 1气压定高 2超声定高 */
/* 定点模式切换
1不定点 2记录返航点 3定点 */
extern u8 height_ctrl_mode,position_ctrl_mode ;
extern u8 position_ctrl_mode_old;

/*----------------------------------------------------------
 + 实现功能：由任务调度调用周期2ms
    接收机控制量数组：Mapped_CH 1000-1500-2000
    数传控制量数组：RX_CH     1000-1500-2000
    无控制量数组：STOP_CH   1000-1500-2000
----------------------------------------------------------*/
extern void Call_RadioContrl(float inner_loop_time);

/*----------------------------------------------------------
 + 实现功能：遥控信号虚拟看门狗，400ms内必须调用一次
 + 调用参数：控制信号类型：1为PWM信号 2为数传
----------------------------------------------------------*/
extern void Call_RadioControl_Sign(u8 ch_mode);

/*----------------------------------------------------------
 + 实现功能：控制模式判断 由任务调度调用周期50ms
----------------------------------------------------------*/
extern void Call_RadioControl_Mode(void);

#endif
/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
