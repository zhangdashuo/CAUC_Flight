/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file driver_mpu6050.c
*@version V1.0
*@date  2018/5/24
*@brief IMU传感器mpu6050驱动文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#include "stm32f4xx.h"
#include "driver_mpu6050.h"
#include "device_mpu6050.h"
#include "device_iic.h"
#include "database.h"
#include "driver_parameter.h"
#include "time.h"

/* 校准时的计算次数 */
#define OFFSET_AV_NUM 	    		50
/* 滑动窗口滤波数值个数 */
#define FILTER_NUM 					10
/* Gyro角速度的单位转换 */
#define TO_ANGLE 					0.06103f

/* MPU6050结构体 */
MPU6050_STRUCT mpu6050;
/* 传感器校准数据累加和 */
s32 sum_temp[7]= {0,0,0,0,0,0,0};
/* 传感器校准临时计数 */
u16 acc_sum_cnt = 0,gyro_sum_cnt = 0;
/* 滤波滑动窗口数组 */
s16 FILT_BUF[ITEMS][(FILTER_NUM + 1)];
/* 滤波滑动窗口数组下标 */
uint8_t filter_cnt = 0,filter_cnt_old = 0;
/* 得出校准后的临时数据 */
float mpu6050_tmp[ITEMS];

/*----------------------------------------------------------
 + 实现功能：MPU6050加速度计、陀螺仪数据校准
----------------------------------------------------------*/
void MPU6050_Data_Offset()
{
    /* 加速度计校准 */
    if(mpu6050.Acc_CALIBRATE == 1)
    {
        /* 计数及累加 */
        acc_sum_cnt++;
        sum_temp[A_X] += mpu6050.Acc_I16.x;
        sum_temp[A_Y] += mpu6050.Acc_I16.y;
        sum_temp[A_Z] += mpu6050.Acc_I16.z - 4096;
        sum_temp[TEM] += mpu6050.Tempreature;

        /* 判断计数符合条件 */
        if( acc_sum_cnt >= OFFSET_AV_NUM )
        {
            /* 计算校验数据 */
            mpu6050.Acc_Offset.x = sum_temp[A_X]/OFFSET_AV_NUM;
            mpu6050.Acc_Offset.y = sum_temp[A_Y]/OFFSET_AV_NUM;
            mpu6050.Acc_Offset.z = sum_temp[A_Z]/OFFSET_AV_NUM;
            mpu6050.Acc_Temprea_Offset = sum_temp[TEM]/OFFSET_AV_NUM;
            /* 清零过程变量 */
            acc_sum_cnt =0;
            mpu6050.Acc_CALIBRATE = 0;
            sum_temp[A_X] = sum_temp[A_Y] = sum_temp[A_Z] = sum_temp[TEM] = 0;
            /* 保存校验数据 */
            Param_SaveAccelOffset(&mpu6050.Acc_Offset);
        }
    }

    /* 陀螺仪校准 */
    if(mpu6050.Gyro_CALIBRATE)
    {
        /* 计数及累加 */
        gyro_sum_cnt++;
        sum_temp[G_X] += mpu6050.Gyro_I16.x;
        sum_temp[G_Y] += mpu6050.Gyro_I16.y;
        sum_temp[G_Z] += mpu6050.Gyro_I16.z;
        sum_temp[TEM] += mpu6050.Tempreature;

        /* 判断计数符合条件 */
        if( gyro_sum_cnt >= OFFSET_AV_NUM )
        {
            /* 计算校验数据 */
            mpu6050.Gyro_Offset.x = (float)sum_temp[G_X]/OFFSET_AV_NUM;
            mpu6050.Gyro_Offset.y = (float)sum_temp[G_Y]/OFFSET_AV_NUM;
            mpu6050.Gyro_Offset.z = (float)sum_temp[G_Z]/OFFSET_AV_NUM;
            mpu6050.Gyro_Temprea_Offset = sum_temp[TEM]/OFFSET_AV_NUM;
            /* 清零过程变量 */
            gyro_sum_cnt =0;
            mpu6050.Gyro_CALIBRATE = 0;
            sum_temp[G_X] = sum_temp[G_Y] = sum_temp[G_Z] = sum_temp[TEM] = 0;
            /* 保存校验数据 */
            Param_SaveGyroOffset(&mpu6050.Gyro_Offset);
        }
    }
}

/*----------------------------------------------------------
 + 实现功能：由任务调度调用周期2ms
----------------------------------------------------------*/
void Call_MPU6050_Data_Prepare(float T)
{
    /* 滤波滑动窗口临时数组 */
    s32 FILT_TMP[ITEMS] = {0,0,0,0,0,0,0};
    /* 原始数据读取临时数组 */
    u8 mpu6050_buffer[14];
    /* 陀螺仪原始数据临时变量 */
    float Gyro_tmp[3];

    /* 判断是否有MPU6050加速度计、陀螺仪数据校准 */
    MPU6050_Data_Offset();

    /*读取原始数据，用mpu6050_buffer[]解析*/
    Call_MPU6050(mpu6050_buffer);
    mpu6050.Acc_I16.x = ((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]) ;
    mpu6050.Acc_I16.y = ((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]) ;
    mpu6050.Acc_I16.z = ((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]) ;
    mpu6050.Gyro_I16.x = ((((int16_t)mpu6050_buffer[ 8]) << 8) | mpu6050_buffer[ 9]) ;
    mpu6050.Gyro_I16.y = ((((int16_t)mpu6050_buffer[10]) << 8) | mpu6050_buffer[11]) ;
    mpu6050.Gyro_I16.z = ((((int16_t)mpu6050_buffer[12]) << 8) | mpu6050_buffer[13]) ;

    /* 陀螺仪原始数据临时变量 */
    Gyro_tmp[0] = mpu6050.Gyro_I16.x ;
    Gyro_tmp[1] = mpu6050.Gyro_I16.y ;
    Gyro_tmp[2] = mpu6050.Gyro_I16.z ;

    /* 读取温度信息并滤波 */
    mpu6050.Tempreature = ((((int16_t)mpu6050_buffer[6]) << 8) | mpu6050_buffer[7]);
    mpu6050.TEM_LPF += 2 *3.14f *T *(mpu6050.Tempreature - mpu6050.TEM_LPF);
    mpu6050.Ftempreature = mpu6050.TEM_LPF/340.0f + 36.5f;

    /* 得出校准后的临时数据 */
    mpu6050_tmp[A_X] = (mpu6050.Acc_I16.x - mpu6050.Acc_Offset.x) ;
    mpu6050_tmp[A_Y] = (mpu6050.Acc_I16.y - mpu6050.Acc_Offset.y) ;
    mpu6050_tmp[A_Z] = (mpu6050.Acc_I16.z - mpu6050.Acc_Offset.z) ;
    mpu6050_tmp[G_X] = Gyro_tmp[0] - mpu6050.Gyro_Offset.x ;
    mpu6050_tmp[G_Y] = Gyro_tmp[1] - mpu6050.Gyro_Offset.y ;
    mpu6050_tmp[G_Z] = Gyro_tmp[2] - mpu6050.Gyro_Offset.z ;

    /* 滑动窗口滤波数组下标移位 */
    if( ++filter_cnt > FILTER_NUM )
    {
        filter_cnt = 0;
        filter_cnt_old = 1;
    }
    else
        filter_cnt_old = (filter_cnt == FILTER_NUM)? 0 : (filter_cnt + 1);

    /* 更新滤波滑动窗口临时数组 */
    FILT_BUF[A_X][filter_cnt] = mpu6050_tmp[A_X];
    FILT_BUF[A_Y][filter_cnt] = mpu6050_tmp[A_Y];
    FILT_BUF[A_Z][filter_cnt] = mpu6050_tmp[A_Z];
    FILT_BUF[G_X][filter_cnt] = mpu6050_tmp[G_X];
    FILT_BUF[G_Y][filter_cnt] = mpu6050_tmp[G_Y];
    FILT_BUF[G_Z][filter_cnt] = mpu6050_tmp[G_Z];
    /* 更新滤波滑动窗口数组 */
    for(u8 i=0; i<FILTER_NUM; i++)
    {
        FILT_TMP[A_X] += FILT_BUF[A_X][i];
        FILT_TMP[A_Y] += FILT_BUF[A_Y][i];
        FILT_TMP[A_Z] += FILT_BUF[A_Z][i];
        FILT_TMP[G_X] += FILT_BUF[G_X][i];
        FILT_TMP[G_Y] += FILT_BUF[G_Y][i];
        FILT_TMP[G_Z] += FILT_BUF[G_Z][i];
    }

    /* 得出处理后的数据 */
    mpu6050.Acc.x  = (float)( FILT_TMP[A_X] )/(float)FILTER_NUM;
    mpu6050.Acc.y  = (float)( FILT_TMP[A_Y] )/(float)FILTER_NUM;
    mpu6050.Acc.z  = (float)( FILT_TMP[A_Z] )/(float)FILTER_NUM;
	/*在这里可以增加校准值但是这个值需要做实验计算*/
	
    mpu6050.Gyro.x  = (float)( FILT_TMP[G_X] )/(float)FILTER_NUM;
    mpu6050.Gyro.y  = (float)( FILT_TMP[G_Y] )/(float)FILTER_NUM;
    mpu6050.Gyro.z  = (float)( FILT_TMP[G_Z] )/(float)FILTER_NUM;

    /* 从数据换算到角速度 */
    mpu6050.Gyro_deg.x = mpu6050.Gyro.x *TO_ANGLE;
    mpu6050.Gyro_deg.y = mpu6050.Gyro.y *TO_ANGLE;
    mpu6050.Gyro_deg.z = mpu6050.Gyro.z *TO_ANGLE;
}

