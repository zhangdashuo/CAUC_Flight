/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.
 + 文件名  device_mpu6050.h
 + 描述    ：IMU传感器mpu6050设备头文件
代码尽量做到逐行注释 代码有问题，及时加群交流 作者有偿支持对开源代码的完善 */
#ifndef _DEVICE_MPU6050_H
#define _DEVICE_MPU6050_H

#include "stm32f4xx.h"

/* IMU传感器硬件故障 */
extern u8 hard_error_mpu6050;

/*----------------------------------------------------------
 + 实现功能：初始化 MPU6050 以进入可用状态
 + 调用参数功能：u16 lpf：设置低通滤波截止频率
----------------------------------------------------------*/
extern void MPU6050_Init(u16);

/*----------------------------------------------------------
 + 实现功能：由任务调度调用周期2ms
 + 调用参数功能：被读取的加速度计陀螺仪数据数组
----------------------------------------------------------*/
extern void Call_MPU6050(u8 *buf14);

#endif
/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
