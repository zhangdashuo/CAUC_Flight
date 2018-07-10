/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file device_ms5611.c
*@version V1.0
*@date  2018/5/24
*@brief 气压计设备文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#include "stm32f4xx.h"
#include "device_ms5611.h"
#include "device_iic.h"

#define MS5611_ADDR             0x77 // MS5611 address
#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command
#define CMD_ADC_CONV            0x40 // ADC conversion command
#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion
#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096
#define CMD_PROM_RD             0xA0 // Prom read command
#define MS5611_OSR				0x08 // CMD_ADC_4096

/*----------------------------------------------------------
 + 实现功能：气压计复位
----------------------------------------------------------*/
void MS5611_Reset(void)
{
    IIC_Write_1Byte(MS5611_ADDR, CMD_RESET, 1);
}

/*----------------------------------------------------------
 + 实现功能：读取气压计PORM存储器 并判断硬件故障
----------------------------------------------------------*/
u8 MS5611_Read_Prom(uint16_t * ms5611_prom8)
{
    uint8_t rxbuf[2] = { 0, 0 };
    u8 check = 0;
    u8 i;

    for (i = 0; i < 8; i++)
    {
        check += IIC_Read_nByte(MS5611_ADDR, CMD_PROM_RD + i * 2, 2, rxbuf); // send PROM READ command
        ms5611_prom8[i] = rxbuf[0] << 8 | rxbuf[1];
    }
    if(check==8)
        return 1;
    else
        return 0;
}

/*----------------------------------------------------------
 + 实现功能：读取测量数据
----------------------------------------------------------*/
void MS5611_Read_measure(u8 * rxbuf)
{
    IIC_Read_nByte( MS5611_ADDR, CMD_ADC_READ, 3, rxbuf );
}

/*----------------------------------------------------------
 + 实现功能：开始读取温度
----------------------------------------------------------*/
void MS5611_Start_T(void)
{
    IIC_Write_1Byte(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D2 + MS5611_OSR, 1);
}

/*----------------------------------------------------------
 + 实现功能：开始读取气压
----------------------------------------------------------*/
void MS5611_Start_P(void)
{
    IIC_Write_1Byte(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D1 + MS5611_OSR, 1);
}

