/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file driver_ms5611.c
*@version V1.0
*@date  2018/5/24
*@brief 气压计驱动文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#include "stm32f4xx.h"
#include "driver_ms5611.h"
#include "device_ms5611.h"
#include "filter.h"
#include "database.h"
#include "time.h"
#define BARO_CAL_CNT 200

/* 气压计计算高度，单位mm(毫米) */
int32_t baroAlt,baroAltOld;
/* 气压计计算速度单位mm/s */
float baro_alt_speed;
/* 读取的气压计PORM存储器数组 */
static uint16_t ms5611_prom8[8];
/* 读取的温度、压强数据数组 */
static uint8_t t_rxbuf[3],p_rxbuf[3];

/*----------------------------------------------------------
 + 实现功能：气压计数据更新 由任务调度调用周期10ms
----------------------------------------------------------*/
int MS5611_Update(void)
{
    /* 气压计状态位 */
    static int state = 0;
    if (state)
    {
        /* 读取上次测量的气压 */
        MS5611_Read_measure(p_rxbuf);
        /* 开始读取温度 */
        MS5611_Start_T();
        /* 气压计高度计算 */
        MS5611_BaroAltCalculate();
        /* 气压计状态位 */
        state = 0;
    }
    else
    {
        /* 读取上次测量的温度 */
        MS5611_Read_measure(t_rxbuf);
        /* 开始读取气压 */
        MS5611_Start_P();
        /* 气压计状态位 */
        state = 1;
    }
    /* 返回状态值 */
    return (state);
}


/* 温度 */
uint32_t ms5611_ut;
/* 气压 */
uint32_t ms5611_up;
/* 气压补偿 */
int32_t baro_Offset=0;
/* 气压计温度 */
float temperature_5611;
/*----------------------------------------------------------
 + 实现功能：气压计高度计算
----------------------------------------------------------*/
void MS5611_BaroAltCalculate(void)
{
    static u8 baro_start;
    int32_t temperature, off2 = 0, sens2 = 0, delt;
    int32_t pressure;
    float alt_3;
    int32_t dT;
    int64_t off;
    int64_t sens;
    static vs32 sum_tmp_5611 = 0;

    /* 气压计温度原始数据 */
    ms5611_ut = (t_rxbuf[0] << 16) | (t_rxbuf[1] << 8) | t_rxbuf[2];
    /* 气压计压强原始数据 */
    ms5611_up = (p_rxbuf[0] << 16) | (p_rxbuf[1] << 8) | p_rxbuf[2];
    /* 气压计原始数据 */
    dT = ms5611_ut - ((uint32_t)ms5611_prom8[5] << 8);
    off = ((uint32_t)ms5611_prom8[2] << 16) + (((int64_t)dT * ms5611_prom8[4]) >> 7);
    sens = ((uint32_t)ms5611_prom8[1] << 15) + (((int64_t)dT * ms5611_prom8[3]) >> 8);
    temperature = 2000 + (((int64_t)dT * ms5611_prom8[6]) >> 23);

    /* 低于20摄氏度补偿 */
    if (temperature < 2000)
    {
        delt = temperature - 2000;
        delt = delt * delt;
        off2 = (5 * delt) >> 1;
        sens2 = (5 * delt) >> 2;
        /* 低于-15摄氏度补偿 */
        if (temperature < -1500)
        {
            delt = temperature + 1500;
            delt = delt * delt;
            off2  += 7 * delt;
            sens2 += (11 * delt) >> 1;
        }
        off  -= off2;
        sens -= sens2;
    }

    /* 计算出气压并转换为高度 */
    pressure = (((ms5611_up * sens ) >> 21) - off) >> 15;
    alt_3 = (101000 - pressure)/1000.0f;
    pressure = 0.0082f *alt_3 * alt_3 *alt_3 + 0.09f *(101000 - pressure)*100.0f ;
    /* 气压计计算高度，单位mm(毫米) */
    baroAlt = pressure - baro_Offset;
    /* 气压计计算速度单位mm/s */
    baro_alt_speed += 5 *0.02 *3.14 *( 50 *( baroAlt - baroAltOld ) - baro_alt_speed );
    /* 保存数据下次使用 */
    baroAltOld = baroAlt;

    /* 第0-99次数据无效，用时20ms*100=2s */
    if( baro_start < 100 )
    {
        /* 计数次数增加 */
        baro_start++;
        /* 气压计计算速度 清零 */
        baro_alt_speed = 0;
        /* 气压计计算高度 清零 */
        baroAlt = 0;
    }

    /* 气压计温度积分滤波 */
    temperature_5611 += 0.01f *( ( 0.01f *temperature ) - temperature_5611 );
}

/*----------------------------------------------------------
 + 实现功能：气压计初始化
----------------------------------------------------------*/
/* 气压计硬件故障 */
u8 hard_error_ms5611;
void MS5611_Init(void)
{
    /* IIC延时等待 */
    Delay_ms(10);
    /* 气压计复位 */
    MS5611_Reset();
    /* IIC延时等待 */
    Delay_ms(3);
    /* 读取气压计PORM存储器 并判断硬件故障 */
    hard_error_ms5611 =  MS5611_Read_Prom(ms5611_prom8);
    /* 开始读取温度 */
    MS5611_Start_T();
}

