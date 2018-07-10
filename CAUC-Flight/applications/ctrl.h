/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file main.c
*@version V1.0
*@date  2018/5/24
*@brief 飞控控制头文件
*@design 无人机研究小组
	     谭政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#ifndef _CTRL_H
#define	_CTRL_H

#include "stm32f4xx.h"
#include "driver_pwm_out.h"
#include "rc.h"
#include "imu.h"
#include "driver_mpu6050.h"

/* PID控制轴枚举体 */
enum
{
    PIDROLL,
    PIDPITCH,
    PIDYAW,
    PID4,
    PID5,
    PID6,
    PIDITEMS
};

/* PID控制结构体 */
typedef struct
{
    xyz_f_t err;
    xyz_f_t err_old;
    xyz_f_t err_i;
    xyz_f_t eliminate_I;
    xyz_f_t err_d;
    xyz_f_t damp;
    xyz_f_t out;
    pid_t 	PID[PIDITEMS];
    xyz_f_t err_weight;
    float FB;
} ctrl_t;

/* 机架类型 */
extern char OP_COPTER ;
/* 角速度控制结构体 */
extern ctrl_t ctrl_angular_velocity;
/* 姿态控制结构体 */
extern ctrl_t ctrl_attitude;
/* 低油门信号判断 */
extern u8 Thr_Low;
/* 滤波后油门数据 */
extern float Thr_Weight;
/* 单个电机的总控制量 */
extern float motor[8];

/*----------------------------------------------------------
 + 实现功能：恢复默认控制幅度
----------------------------------------------------------*/
extern void Ctrl_Para_Init(void);

/*----------------------------------------------------------
 + 实现功能：姿态PID控制角速度 由任务调度调用周期5ms
 + 调用参数：两次调用间隔
----------------------------------------------------------*/
extern void CTRL_attitude(float);

/*----------------------------------------------------------
 + 实现功能：角速度电机输出量 由任务调度调用周期2ms
 + 调用参数：两次调用间隔
----------------------------------------------------------*/
extern void CTRL_angular_velocity(float);

#endif

