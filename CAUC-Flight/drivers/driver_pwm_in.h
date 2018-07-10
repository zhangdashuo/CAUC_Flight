/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.
 + 文件名  driver_pwm_in.h
 + 描述    ：PWM输入捕获驱动头文件
代码尽量做到逐行注释 代码有问题，及时加群交流 作者有偿支持对开源代码的完善 */
#ifndef _DRIVER_PWM_IN_H_
#define _DRIVER_PWM_IN_H_

#include "stm32f4xx.h"
#include "device_pwm_in.h"

/* 8通道输入PWM数据的数组1000-2000微秒的高电平信号 */
extern u16 Rc_Pwm_In[8];

#endif
/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
