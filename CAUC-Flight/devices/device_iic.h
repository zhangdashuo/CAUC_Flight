/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file main.c
*@version V1.0
*@date  2018/5/24
*@brief IIC设备头文件
*@design 无人机研究小组
	     谭政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#ifndef _DEVICE_IIC_H
#define	_DEVICE_IIC_H

#include "stm32f4xx.h"

/* IIC延时长短 */
extern volatile u8 I2C_FastMode;

/*----------------------------------------------------------
 + 实现功能：IIC设备初始化
----------------------------------------------------------*/
void I2c_Device_Init(void);

/*----------------------------------------------------------
 + 实现功能：IIC写入单字节数据
 + 调用参数功能：设备，寄存器，数据
 + 返回参数功能：0正常 1异常
----------------------------------------------------------*/
extern u8 IIC_Write_1Byte(u8 SlaveAddress,u8 REG_Address,u8 REG_data);

/*----------------------------------------------------------
 + 实现功能：IIC读取单字节数据
 + 调用参数功能：设备，寄存器，数据
 + 返回参数功能：0正常 1异常
----------------------------------------------------------*/
extern u8 IIC_Read_1Byte(u8 SlaveAddress,u8 REG_Address,u8 *REG_data);

/*----------------------------------------------------------
 + 实现功能：IIC写入多字节数据
 + 调用参数功能：设备，寄存器，数据长度，数据
 + 返回参数功能：0正常 1异常
----------------------------------------------------------*/
extern u8 IIC_Write_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);

/*----------------------------------------------------------
 + 实现功能：IIC读取多字节数据
 + 调用参数功能：设备，寄存器，数据长度，数据
 + 返回参数功能：0正常 1异常
----------------------------------------------------------*/
extern u8 IIC_Read_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);

#endif

