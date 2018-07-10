/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file device_ak8975.c
*@version V1.0
*@date  2018/5/24
*@brief 磁力计（电子罗盘）设备文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#include "device_ak8975.h"
#include "stm32f4xx.h"
#include "device_iic.h"

/* AK8975的寄存器 */
#define AK8975_ADDRESS 0x0c
#define AK8975_WIA     0x00
#define AK8975_HXL     0x03
#define AK8975_HXH     0x04
#define AK8975_HYL     0x05
#define AK8975_HYH     0x06
#define AK8975_HZL     0x07
#define AK8975_HZH     0x08
#define AK8975_CNTL    0x0A

/*----------------------------------------------------------
 + 实现功能：磁力计采样触发
 + 返回参数：磁力计运行标识
----------------------------------------------------------*/
uint8_t AK8975_IS_RUN(void)
{
    return IIC_Write_1Byte(AK8975_ADDRESS,AK8975_CNTL,0x01);
}

/*----------------------------------------------------------
 + 实现功能：获取磁力计X方向数据
 + 返回参数：磁力计X轴原始数据
----------------------------------------------------------*/
int16_t AK8975_Read_Mag_X(void)
{
    int16_t mag_temp_X;
    u8 ak8975_buffer[2]; //接收数据缓存
    I2C_FastMode = 0;
    IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HXL,&ak8975_buffer[0]);
    IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HXH,&ak8975_buffer[1]);
    mag_temp_X = ((((int16_t)ak8975_buffer[1]) << 8) | ak8975_buffer[0]) ;
    return mag_temp_X;
}

/*----------------------------------------------------------
 + 实现功能：获取磁力计Y方向数据
 + 返回参数：磁力计Y轴原始数据
----------------------------------------------------------*/
int16_t AK8975_Read_Mag_Y(void)
{
    int16_t mag_temp_Y;
    u8 ak8975_buffer[2]; //接收数据缓存
    I2C_FastMode = 0;
    IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HYL,&ak8975_buffer[0]);
    IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HYH,&ak8975_buffer[1]);
    mag_temp_Y = ((((int16_t)ak8975_buffer[1]) << 8) | ak8975_buffer[0]) ;
    return mag_temp_Y;
}

/*----------------------------------------------------------
 + 实现功能：获取磁力计Z方向数据
 + 返回参数：磁力计Z轴原始数据
----------------------------------------------------------*/
int16_t AK8975_Read_Mag_Z(void)
{
    int16_t mag_temp_Z;
    u8 ak8975_buffer[2]; //接收数据缓存
    I2C_FastMode = 0;
    IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HZL,&ak8975_buffer[0]);
    IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HZH,&ak8975_buffer[1]);
    mag_temp_Z = ((((int16_t)ak8975_buffer[1]) << 8) | ak8975_buffer[0]) ;
    return mag_temp_Z;
}

