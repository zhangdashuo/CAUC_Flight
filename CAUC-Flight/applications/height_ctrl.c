/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file height_ctrl.c
*@version V1.0
*@date  2018/5/24
*@brief 高度控制文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#include "stm32f4xx.h"
#include "height_ctrl.h"
#include "driver_ms5611.h"
#include "driver_ultrasonic.h"
#include "rc.h"
#include "ctrl.h"
#include "mymath.h"
#include "filter.h"
#include "position_control.h"

/* 气压计速度 滑动窗口滤波数组 数量 */
#define BARO_SPEED_NUM 100
/* 油门控制量转换高度控制 */
#define EXP_Z_SPEED  ( 4.0f *my_deathzoom((thr-500),50) )

/* 超声波定高标准比例高度速度控制 单位 mm/s */
#define ULTRA_SPEED 		 0.625f
/* 超声波定高最大高度上升速度控制 单位 mm/s */
#define Max_ULTRA_SPEED 		 250.0f
/* 超声波定高最大高度下降速度控制 单位 mm/s */
#define Min_ULTRA_SPEED 		 -150.0f

/* 超声波定高最大高度位置控制 单位 mm */
#define ULTRA_MAX_HEIGHT 1500
/* 超声波定高最小高度位置控制 单位 mm */
#define ULTRA_MIN_HEIGHT 200

/* 超声波定高积分限幅 单位 mm */
#define ULTRA_INT        300

/* 加速度在各方向的分量 */
float wz_acc,north_acc,west_acc;
/* 气压计速度 滑动窗口滤波数组 单位mm/s */
float baro_speed_arr[BARO_SPEED_NUM + 1];
/* 气压计速度 滑动窗口滤波数组下标 */
u16 baro_cnt[2];
/*向上方向的加速度 单位mm/s2，速度 单位mm/s，上一次速度 */
float wz_acc_mms2,wz_speed,wz_speed_old;
/*向北方向的加速度 单位mm/s2，速度 单位mm/s，上一次速度 */
float north_acc_mms2,north_speed,north_speed_old;
/*向西方向的加速度 单位mm/s2，速度 单位mm/s，上一次速度 */
float west_acc_mms2,west_speed,west_speed_old;

/* 最初启动气压定高 */
float start_height;
/* 高度速率PID控制量结构体 */
_st_height_pid_v wz_speed_pid_v;
/* 高度速率PID参数结构体 */
_st_height_pid wz_speed_pid;
/* 气压计高度速率 */
float baro_speed;
/* 定高高度速率PID输出量 */
float height_ctrl_out;

/* 气压计有接收到数据状态 */
u8 baro_ctrl_start;
/* 气压计高度 上一次高度 */
float baro_height,baro_height_old;
/* 10倍气压计高度差 */
float baro_measure;
/* 超声波位置PID控制量结构体 */
_st_height_pid_v ultra_ctrl;
/* 超声波位置PID参数结构体 */
_st_height_pid ultra_pid;

/* 超声波期望高度速率 *//* 超声波期望高度位置 */
float exp_height_speed,exp_height;
/* 超声波高度速率 */
float ultra_speed;
/* 超声波距离积分滤波 */
float ultra_dis_lpf;
/* 超声波定高高度保持PID输出量 */
float ultra_ctrl_out;

/*----------------------------------------------------------
 + 实现功能：油门控制高度
 + 调用参数：两次调用时间间隔，油门信号量
----------------------------------------------------------*/
void Height_Ctrl(float T,float thr)
{
    /* 开始高度控制 */
    static u8 height_ctrl_start_f;
    /* 开始高度控制延时 */
    static u16 hc_start_delay;
    /* 超声波定高计时 */
    static u8 hs_ctrl_cnt;

    /* 是否已经开始高度控制 */
    if ( height_ctrl_start_f )
    {
        /* 向上加速度分量 */
        wz_acc += ( 1 / ( 1 + 1 / ( 20 *3.14f *T ) ) ) *( (reference_v.z *mpu6050.Acc.z + reference_v.x *mpu6050.Acc.x + reference_v.y *mpu6050.Acc.y - 4096 ) - wz_acc );
        /* 向北加速度分量 */
        north_acc += ( 1 / ( 1 + 1 / ( 20 *3.14f *T ) ) ) *( (north.x *mpu6050.Acc.x + north.y *mpu6050.Acc.y + north.z *mpu6050.Acc.z  ) - north_acc );
        /* 向西加速度分量 */
        west_acc += ( 1 / ( 1 + 1 / ( 20 *3.14f *T ) ) ) *( (west.x *mpu6050.Acc.x + west.y *mpu6050.Acc.y + west.z *mpu6050.Acc.z ) - west_acc );

        /* 气压计速度 滑动窗口滤波 单位mm/s*/
        Moving_Average( (float)( baro_alt_speed *10),baro_speed_arr,BARO_SPEED_NUM, baro_cnt ,&baro_speed );
        /* 气压计定高 */
        if( height_ctrl_mode == 1)
        {
            /* 气压计有接收到数据状态 */
            if(baro_ctrl_start==1)
            {
                /* 气压计高度速度控制 */
                height_speed_ctrl(0.02f,thr,( EXP_Z_SPEED ),baro_speed);
                /* 重重气压计接收到数据状态标志位 */
                baro_ctrl_start = 0;
                /* 气压计高度位置控制 */
                Baro_Ctrl(0.02f,thr);
            }
        }
        /* 超声波定高 */
        else if( height_ctrl_mode == 2)
        {
            /* 通过计时使周期100ms */
            hs_ctrl_cnt++;
            hs_ctrl_cnt = hs_ctrl_cnt%10;
            /* 判断计时使周期 */
            if(hs_ctrl_cnt == 0)
            {
                /* 超声波高度速度控制 */
                height_speed_ctrl(0.02f,thr,0.4f*ultra_ctrl_out,ultra_speed);
            }
            /* 超声波有接收到数据状态 */
            if( ultra_state == 0 )
            {
                /* 超声波高度位置控制 */
                Ultra_Ctrl(0.1f,thr);
                /* 重重超声波接收到数据状态标志位 */
                ultra_state = -1;
            }
        }
        /* 开启了定高模式 */
        if(height_ctrl_mode)
            /* 高度控制PID输出 */
            height_ctrl_out = wz_speed_pid_v.pid_out;
        /* 没有开启高度控制 */
        else
            /* 高度控制油门输出 */
            height_ctrl_out = thr;
    }
    /* 还没有开始高度控制 */
    else if(height_ctrl_start_f == 0)
    {
        /* 水平放置立即开始高度控制 */
        if( mpu6050.Acc.z > 4000 )
            height_ctrl_start_f = 1;
        /* 倾斜放置需要延迟开始 */
        else if( ++hc_start_delay > 500 )
            height_ctrl_start_f = 1;
    }
}

/*----------------------------------------------------------
 + 实现功能：气压定高的PID参数初始化
----------------------------------------------------------*/
void WZ_Speed_PID_Init()
{
    wz_speed_pid.kp = pid_setup.groups.hc_sp.kp;
    wz_speed_pid.kd = pid_setup.groups.hc_sp.kd;
    wz_speed_pid.ki = pid_setup.groups.hc_sp.ki;
}

/*----------------------------------------------------------
 + 实现功能：高度的速度控制
 + 调用参数：两次调用时间间隔 遥控器油门控制量 期望垂直方向速度 垂直方向速度
----------------------------------------------------------*/
void height_speed_ctrl(float T,float thr,float exp_z_speed,float h_speed)
{
    /* 油门控制量积分滤波 */
    static float thr_lpf;
    /* 油门控制量限幅转化为高度控制量 */
    float height_thr;
    /* 单一方向的加速度过程量 */
    static float hc_acc_i,wz_speed_0,wz_speed_1;
    static float north_acc_i,north_speed_0,north_speed_1;
    static float west_acc_i,west_speed_0,west_speed_1;

    /* 油门控制量限幅转化为高度控制量 */
    height_thr = LIMIT( 2 * thr , 0, 600 );
    /* 油门控制量积分滤波 */
    thr_lpf += ( 1 / ( 1 + 1 / ( 0.5f *3.14f *T ) ) ) *( height_thr - thr_lpf );

    /* 向上的加速度积分数据和高度融合 */
    /* 没有重力数的据提取 */
    wz_acc_mms2 = (wz_acc/4096.0f) *10000 + hc_acc_i;
    /* 死区范围外数据积分 */
    wz_speed_0 += my_deathzoom( (wz_acc_mms2 ) ,100) *T;
    /* 加速度数据积分 */
    hc_acc_i += 0.4f *T *( (wz_speed - wz_speed_old)/T - wz_acc_mms2 );
    /* 加速度数据积分限幅 */
    hc_acc_i = LIMIT( hc_acc_i, -500, 500 );
    /* 高度数据积分滤波 */
    wz_speed_0 += ( 1 / ( 1 + 1 / ( 0.1f *3.14f *T ) ) ) *( h_speed - wz_speed_0  ) ;
    /* 计算出的速度赋值 */
    wz_speed_1 = wz_speed_0;
    /* 死区范围外数据有效 */
    if( ABS( wz_speed_1 ) < 50 )wz_speed_1 = 0;
    /* 记录上一次数据 */
    wz_speed_old = wz_speed;
    /* 计算出的速度赋值 */
    wz_speed = wz_speed_1;

    /* 向北的加速度积分数据和纬度融合 */
    /* 没有重力数的据提取 */
    north_acc_mms2 = (north_acc/4096.0f) *10000 + north_acc_i;
    /* 死区范围外数据积分 */
    north_speed_0 += my_deathzoom( (north_acc_mms2 ) ,1000) * T;
    /* 加速度数据积分 */
    north_acc_i += 0.4f *T *( (north_speed - north_speed_old)/T - north_acc_mms2 );
    /* 加速度数据积分限幅 */
    north_acc_i = LIMIT( north_acc_i, -500, 500 );
    /* 高度数据积分滤波 */
    north_speed_0 += ( 1 / ( 1 + 1 / ( 0.1f *3.14f *T ) ) ) *( speed_latitude - north_speed_0  ) ;
    /* 计算出的速度赋值 */
    north_speed_1 = north_speed_0;
    /* 死区范围外数据有效 */
    if( ABS( north_speed_1 ) < 50 )north_speed_1 = 0;
    /* 记录上一次数据 */
    north_speed_old = north_speed;
    /* 计算出的速度赋值 */
    north_speed = north_speed_1;

    /* 向西的加速度积分数据和经度融合 */
    /* 没有重力的数据提取 */
    west_acc_mms2 = (west_acc/4096.0f) *10000 + west_acc_i;
    /* 死区范围外数据积分 */
    west_speed_0 += my_deathzoom( (west_acc_mms2 ) ,1000) * T;
    /* 加速度数据积分 */
    west_acc_i += 0.4f *T *( (west_speed - west_speed_old)/T - west_acc_mms2 );
    /* 加速度数据积分限幅 */
    west_acc_i = LIMIT( west_acc_i, -500, 500 );
    /* 高度数据积分滤波 */
    west_speed_0 += ( 1 / ( 1 + 1 / ( 0.1f *3.14f *T ) ) ) *( speed_longitude - west_speed_0  ) ;
    /* 计算出的速度赋值 */
    west_speed_1 = west_speed_0;
    /* 死区范围外数据有效 */
    if( ABS( west_speed_1 ) < 50 )west_speed_1 = 0;
    /* 记录上一次数据 */
    west_speed_old = west_speed;
    /* 计算出的速度赋值 */
    west_speed = west_speed_1;

    /* 期望速度的比例 */
    wz_speed_pid_v.err = wz_speed_pid.kp *( exp_z_speed - wz_speed );
    /* 期望速度的微分 */
    wz_speed_pid_v.err_d = 0.002f/T *10*wz_speed_pid.kd * (-wz_acc_mms2) *T;
    /* 期望速度的积分 */
    wz_speed_pid_v.err_i += wz_speed_pid.ki *wz_speed_pid.kp *( exp_z_speed - h_speed ) *T;
    /* 期望速度的积分限幅 */
    wz_speed_pid_v.err_i = LIMIT(wz_speed_pid_v.err_i,-Thr_Weight *150,Thr_Weight *150);  //将上限减小chage by zhangshuo 原值300  
    /* 期望速度的PID和 */
    wz_speed_pid_v.pid_out = thr_lpf + Thr_Weight *LIMIT((wz_speed_pid.kp *exp_z_speed + wz_speed_pid_v.err + wz_speed_pid_v.err_d + wz_speed_pid_v.err_i),-200,200);//原值为（300）
    /* 记录上一次的期望速度的 */
    wz_speed_pid_v.err_old = wz_speed_pid_v.err;
}

/*----------------------------------------------------------
 + 实现功能：气压计定高控制
 + 调用参数：两次调用时间间隔 遥控器油门控制量
----------------------------------------------------------*/
void Baro_Ctrl(float T,float thr)
{
    /* 最初启动气压定高 */
    if( (s16)start_height == 0 )
        /* 记录气压计高度 */
        start_height = baroAlt;
    /* 10倍气压计高度差 */
    baro_measure = 10 *( baroAlt - start_height );
    /* 气压计高度与加速度积分 */
    baro_height += T *wz_speed;
    /* 气压计高度与气压计高度差积分 */
    baro_height += 0.2f *3.14f *T *(baro_measure - baro_height);
}

/*----------------------------------------------------------
 + 实现功能：超声波定高的PID参数初始化
----------------------------------------------------------*/
void Ultra_PID_Init()
{
    ultra_pid.kp = pid_setup.groups.hc_height.kp;
    ultra_pid.kd = pid_setup.groups.hc_height.kd;
    ultra_pid.ki = pid_setup.groups.hc_height.ki;
}

/*----------------------------------------------------------
 + 实现功能：超声波定高控制
 + 调用参数：两次调用时间间隔 遥控器油门控制量
----------------------------------------------------------*/
void Ultra_Ctrl(float T,float thr)
{
    /* 超声波速度中位数 *//* 超声波距离中位数滤波 */
    float ultra_sp_tmp,ultra_dis_tmp;

    /* 超声波期望速度 */
    exp_height_speed = ULTRA_SPEED * my_deathzoom_2(thr - 500,50) ;
    /* 超声波期望速度限幅 */
    exp_height_speed = LIMIT(exp_height_speed ,Min_ULTRA_SPEED,Max_ULTRA_SPEED);

    /* 超声波定高最大高度限制 1500mm */
    if( exp_height > ULTRA_MAX_HEIGHT )
    {
        if( exp_height_speed > 0 )
            exp_height_speed = 0;
    }
    /* 超声波定高最小高度限制 20mm */
    else if( exp_height < ULTRA_MIN_HEIGHT )
    {
        if( exp_height_speed < 0 )
            exp_height_speed = 0;
    }
//    /* 超声波期望高度 */
//    exp_height += exp_height_speed *T;
	
    /* 超声波期望高度  chage by zhangshuo*/
    exp_height = exp_height_speed *6;

    /* 超声波定高最大高度限制 1500mm */
    if( exp_height > ULTRA_MAX_HEIGHT )
    {
        exp_height = ULTRA_MAX_HEIGHT;
    }
    /* 超声波定高最小高度限制 20mm */
    else if( exp_height < ULTRA_MIN_HEIGHT )
    {
        exp_height = ULTRA_MIN_HEIGHT;
    }
#if 0
    /* 低油门 */
    if( thr < 100 )
        /* 超声波期望高度缓慢降低 */
        exp_height += ( 1 / ( 1 + 1 / ( 0.2f *3.14f *T ) ) ) *( -exp_height);
#endif
    /* 超声波速度中位数 */
    ultra_sp_tmp = Moving_Median(0,5,ultra_delta/T);

    /* 超声波数据速度小于100mm/s */
    if( ABS(ultra_sp_tmp) < 100 )
        /* 超声波速度积分滤波 */
        ultra_speed += ( 1 / ( 1 + 1 / ( 4 *3.14f *T ) ) ) * ( (float)(ultra_sp_tmp) - ultra_speed );
    /* 超声波数据速度大于100mm/s */
    else
        /* 超声波速度积分滤波 */
        ultra_speed += ( 1 / ( 1 + 1 / ( 1.0f *3.14f *T ) ) ) * ( (float)(ultra_sp_tmp) - ultra_speed );

    /* 超声波距离中位数滤波 */
    ultra_dis_tmp = Moving_Median(1,5,ultra_distance);

    /* 超声波距离变化率小于100mm */
    if( ABS(ultra_dis_tmp - ultra_dis_lpf) < 100 )
        /* 超声波距离积分滤波 */
        ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 4.0f *3.14f *T ) ) ) *(ultra_dis_tmp - ultra_dis_lpf) ;
    /* 超声波距离变化率小于200mm */
    else if( ABS(ultra_dis_tmp - ultra_dis_lpf) < 200 )
        /* 超声波距离积分滤波 */
        ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 2.2f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;
    /* 超声波距离变化率小于300mm */
    else if( ABS(ultra_dis_tmp - ultra_dis_lpf) < 400 )
        /* 超声波距离积分滤波 */
        ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 1.2f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;
    /* 超声波距离变化率较大 */
    else
        /* 超声波距离积分滤波 */
        ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 0.6f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;

    /* 超声波高度比例 */
    ultra_ctrl.err = ( ultra_pid.kp*(exp_height - ultra_dis_lpf) );
    /* 超声波高度积分 */
    ultra_ctrl.err_i += ultra_pid.ki *ultra_ctrl.err *T;
    /* 超声波高度积分限幅 */
    ultra_ctrl.err_i = LIMIT(ultra_ctrl.err_i,-Thr_Weight *ULTRA_INT,Thr_Weight *ULTRA_INT);
    /* 超声波高度微分 */
    ultra_ctrl.err_d = ultra_pid.kd *( 0.6f *(-wz_speed*T) + 0.4f *(ultra_ctrl.err - ultra_ctrl.err_old) );
    /* 超声波高度控制PID求和 */
    ultra_ctrl.pid_out = ultra_ctrl.err + ultra_ctrl.err_i + ultra_ctrl.err_d;
    /* 超声波高度控制PID限度 */
    ultra_ctrl.pid_out = LIMIT(ultra_ctrl.pid_out,-500,500);
    /* 超声波高度控制PID输出量 */
    ultra_ctrl_out = ultra_ctrl.pid_out;
    /* 记录上一次数据 */
    ultra_ctrl.err_old = ultra_ctrl.err;
}

