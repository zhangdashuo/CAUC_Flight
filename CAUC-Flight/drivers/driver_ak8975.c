/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file driver_ak8975.c
*@version V1.0
*@date  2018/5/24
*@brief 磁力计（电子罗盘）驱动文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#include "stm32f4xx.h"
#include "device_ak8975.h"
#include "driver_ak8975.h"
#include "database.h"
#include "driver_parameter.h"
#include "mymath.h"

/* 校准磁力计时持续时间 */
#define CALIBRATING_MAG_CYCLES   3000
/* 磁力计均值滤波数组 */
#define MAG_FILTER_NUM 20

/* 磁力计数组：采样值,偏移值,纠正后的值 */
ak8975_t ak8975 = { {0,0,0},{-1,-1,-1},{0,0,0} };
/* 磁力计硬件故障 */
u8 hard_error_ak8975;
/* 磁力计校准标识 */
u8 Mag_CALIBRATED = 0;

/*----------------------------------------------------------
 + 实现功能：磁力计采样触发
 + 返回值：磁力计运行状态
----------------------------------------------------------*/
uint8_t AK8975_IS_EXIST(void)
{
    return AK8975_IS_RUN();
}

/*----------------------------------------------------------
 + 实现功能：判断是否磁力计校准
----------------------------------------------------------*/
void AK8975_Set_CalOffset(void)
{
    /* 校准前的缺省值 */
    static xyz_f_t	MagMAX = { -100 , -100 , -100 }, MagMIN = { 100 , 100 , 100 };
    /* 校准时间计数 */
    static uint16_t cnt_m=0;

    /* 判断是否磁力计校准 */
    if(Mag_CALIBRATED)
    {
        /* 校准时间计数 */
        cnt_m++;
        /* 保存单一方向最值 */
        if(ABS(ak8975.Mag_Adc.x)<400)
        {
            MagMAX.x = MAX(ak8975.Mag_Adc.x, MagMAX.x);
            MagMIN.x = MIN(ak8975.Mag_Adc.x, MagMIN.x);
        }
        /* 保存单一方向最值 */
        if(ABS(ak8975.Mag_Adc.y)<400)
        {
            MagMAX.y = MAX(ak8975.Mag_Adc.y, MagMAX.y);
            MagMIN.y = MIN(ak8975.Mag_Adc.y, MagMIN.y);
        }
        /* 保存单一方向最值 */
        if(ABS(ak8975.Mag_Adc.z)<400)
        {
            MagMAX.z = MAX(ak8975.Mag_Adc.z, MagMAX.z);
            MagMIN.z = MIN(ak8975.Mag_Adc.z, MagMIN.z);
        }
        /* 校准时间到 */
        if(cnt_m == CALIBRATING_MAG_CYCLES)
        {
            /* 保存校准后的数据 */
            ak8975.Mag_Offset.x = (MagMAX.x + MagMIN.x)/2;
            ak8975.Mag_Offset.y = (MagMAX.y + MagMIN.y)/2;
            ak8975.Mag_Offset.z = (MagMAX.z + MagMIN.z)/2;
            /* 保存数据 */
            Param_SaveMagOffset(&ak8975.Mag_Offset);
            /* 校准、校准时间清零 */
            cnt_m = 0;
            Mag_CALIBRATED = 0;
        }
    }
}

/*----------------------------------------------------------
 + 实现功能：由任务调度调用周期10ms
----------------------------------------------------------*/
void Call_AK8975(void)
{
    /* 滤波滑动窗口临时数组 */
    static xyz_f_t Mag_Adc_fill[MAG_FILTER_NUM];
    /* 滤波滑动窗口总和 */
    static xyz_f_t Mag_Adc_fill_total;
    /* 滤波滑动窗口数组下标 */
    static char temp_i;

    /* 磁力计采样原始数据 */
    ak8975.Mag_Adc.x =   AK8975_Read_Mag_X();
    ak8975.Mag_Adc.y =   AK8975_Read_Mag_Y();
    ak8975.Mag_Adc.z =   AK8975_Read_Mag_Z();

    /* 更新滤波滑动窗口临时数组 */
    Mag_Adc_fill[temp_i].x = ak8975.Mag_Adc.x;
    Mag_Adc_fill[temp_i].y = ak8975.Mag_Adc.y;
    Mag_Adc_fill[temp_i].z = ak8975.Mag_Adc.z;
    /* 滑动窗口滤波数组下标移位 */
    temp_i++;
    if(temp_i>=MAG_FILTER_NUM)temp_i=0;

    /* 更新滑动窗口滤波数据总和 */
    Mag_Adc_fill_total.x+=ak8975.Mag_Adc.x;
    Mag_Adc_fill_total.y+=ak8975.Mag_Adc.y;
    Mag_Adc_fill_total.z+=ak8975.Mag_Adc.z;
    Mag_Adc_fill_total.x-=Mag_Adc_fill[temp_i].x;
    Mag_Adc_fill_total.y-=Mag_Adc_fill[temp_i].y;
    Mag_Adc_fill_total.z-=Mag_Adc_fill[temp_i].z;

    /* 得出处理后的数据 */
    ak8975.Mag_Val.x = Mag_Adc_fill_total.x / MAG_FILTER_NUM;
    ak8975.Mag_Val.y = Mag_Adc_fill_total.y / MAG_FILTER_NUM;
    ak8975.Mag_Val.z = Mag_Adc_fill_total.z / MAG_FILTER_NUM;

    /* 减去偏置校准 */
    ak8975.Mag_Val.x = (ak8975.Mag_Val.x - ak8975.Mag_Offset.x) ;
    ak8975.Mag_Val.y = (ak8975.Mag_Val.y - ak8975.Mag_Offset.y) ;
    ak8975.Mag_Val.z = (ak8975.Mag_Val.z - ak8975.Mag_Offset.z) ;

    /* 判断是否磁力计校准 */
    AK8975_Set_CalOffset();
    /* 磁力计采样触发 */
    AK8975_IS_RUN();
}

