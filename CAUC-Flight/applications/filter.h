/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.
 + 文件名  ：filter.h
 + 描述    ：数据滤波头文件
代码尽量做到逐行注释 代码有问题，及时加群交流 作者有偿支持对开源代码的完善 */
#ifndef __FILTER_H
#define __FILTER_H

#include "driver_parameter.h"

/*----------------------------------------------------------
 + 实现功能：float类型数据滑动窗口滤波
 + 调用参数：in：加入的数据 moavarray[]：滑动窗口数组 len：求取数据个数 fil_cnt[2]：数组下标
 + 调用参数：*out ：算出的中位数
----------------------------------------------------------*/
extern void Moving_Average(float in,float moavarray[],u16 len ,u16 fil_cnt[2],float *out);

/*----------------------------------------------------------
 + 实现功能：float类型数据求中位数
 + 调用参数：item：项目 width_num：求取数据个数 in：加入的数据
 + 返回参数：算出的中位数
----------------------------------------------------------*/
extern float Moving_Median(u8 item,u8 width_num,float in);

/*----------------------------------------------------------
 + 实现功能：int类型数据求中位数
 + 调用参数：item：项目 width_num：求取数据个数 in：加入的数据
 + 返回参数：算出的中位数
----------------------------------------------------------*/
extern int Moving_Median_int(u8 item,u8 width_num,int in_int);

#endif
/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
