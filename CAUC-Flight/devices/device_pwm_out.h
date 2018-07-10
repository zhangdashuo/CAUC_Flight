/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.
 + 文件名  ：device_pwm_out.h
 + 描述    ：PWM输出设备头文件
代码尽量做到逐行注释 代码有问题，及时加群交流 作者有偿支持对开源代码的完善 */
#ifndef _DEVICE_PWM_OUT_H_
#define _DRVICE_PWM_OUT_H_

#include "stm32f4xx.h"

/*----------------------------------------------------------
 + 实现功能：PWM输出初始化
 + 调用参数功能：uint16_t hz：PWM输出的频率
----------------------------------------------------------*/
extern void PWM_Out_Init(uint16_t hz);//400hz

#endif
/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
