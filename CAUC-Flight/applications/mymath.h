/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.
 + 文件名  ：mymath.h
 + 描述    ：数学函数头文件
代码尽量做到逐行注释 代码有问题，及时加群交流 作者有偿支持对开源代码的完善 */
#ifndef __MYMATH_H__
#define __MYMATH_H__

#include "stm32f4xx.h"
#include "arm_math.h"
#include "math.h"

/* 数学函数简单宏定义函数 */
#define ABS(x) ( (x)>0?(x):-(x) )
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

/*----------------------------------------------------------
 + 实现功能：死区控制：减去zoom值
----------------------------------------------------------*/
extern float my_deathzoom(float x,float zoom);

/*----------------------------------------------------------
 + 实现功能：死区控制：绝对值zoom内清零
----------------------------------------------------------*/
extern float my_deathzoom_2(float x,float zoom);

/*----------------------------------------------------------
 + 实现功能：角度范围控制在+-180角度
----------------------------------------------------------*/
extern float To_180_degrees(float x);

/*----------------------------------------------------------
 + 实现功能：快速反正切计算
----------------------------------------------------------*/
extern float fast_atan2(float y, float x);

#endif
/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
