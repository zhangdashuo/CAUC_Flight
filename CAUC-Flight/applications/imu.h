/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.
 + 文件名  ：imu.h
 + 描述    ：姿态解算头文件
代码尽量做到逐行注释 代码有问题，及时加群交流 作者有偿支持对开源代码的完善 */
#ifndef _IMU_H_
#define	_IMU_H_

#include "stm32f4xx.h"
#include "driver_parameter.h"

/* 数据处理过程量结构体 */
typedef struct
{
    xyz_f_t err;
    xyz_f_t err_tmp;
    xyz_f_t err_lpf;
    xyz_f_t err_Int;
    xyz_f_t g;

} ref_t;
/* 加速度：由下向上方向的加速度在加速度计的分量 */
extern xyz_f_t reference_v;
/* 加速度：由南向北方向的加速度在加速度计的分量 *//* 加速度：由东向西方向的加速度在加速度计的分量 */
extern xyz_f_t north,west;
/* 最终计算出的姿态 单位 角度 */
extern float IMU_Roll,IMU_Pitch,IMU_Yaw;

/*----------------------------------------------------------
 + 实现功能：由任务调度调用周期5ms
 + 调用参数：两次调用时间差的一半
----------------------------------------------------------*/
extern void Call_IMUupdate(float half_T);

#endif
/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
