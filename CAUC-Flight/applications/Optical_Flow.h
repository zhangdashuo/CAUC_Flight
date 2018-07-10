/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file Optical_Flow.h
*@version V1.0
*@date  2018/5/24
*@brief 光流驱动头文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#ifndef _OPTICAL_FLOW_H
#define	_OPTICAL_FLOW_H

#include "stm32f4xx.h"

/* 读取X偏移量原始数据  */
extern int Optical_flow_org_x;
/* 读取Y偏移量原始数据 */
extern int Optical_flow_org_y;
/* 读取Confidence原始数据 */
extern int Optical_flow_confidence;

/* 换算到X像素速度  光流像素要结合高度才能算出速度 值范围约+-200.000 */
extern float Optical_flow_x;
/* 换算到Y像素速度  光流像素要结合高度才能算出速度 值范围约+-200.000 */
extern float Optical_flow_y;
/* 读取画面不变率 值范围0-100 */
extern float Optical_flow_con;

/* 光流通过陀螺和超声波补偿后的速度 */
extern float  Flow_Comp_x,Flow_Comp_y;
/* 光流通过超声波补偿后的速度 */
extern float  no_compen_x_speed,no_compen_y_speed;
/* 光流积分位移 */
extern float  OF_Offset_x,OF_Offset_y;

/*----------------------------------------------------------
 + 实现功能：数据接收并保存
 + 调用参数：接收到的单字节数据
----------------------------------------------------------*/
extern void Optical_Flow_Receive_Prepare(u8 data);

/*----------------------------------------------------------
 + 实现功能：光流初始化
----------------------------------------------------------*/
extern void Optical_Flow_init(void);

#endif

