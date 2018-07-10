/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file rc.c
*@version V1.0
*@date  2018/5/24
*@brief 遥控器通道数据处理文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#include "stm32f4xx.h"
#include "rc.h"
#include "mymath.h"
#include "driver_mpu6050.h"
#include "driver_GPS.h"

/* 宏定义遥控器解锁方式：
0：默认解锁方式，1：外八解锁方式 */
#define USE_TOE_IN_UNLOCK    0

/* 基础控制模式，不进行位置控制 */
#define BESE_Control

/* 8通道输入PWM数据的数组1000-2000微秒的高电平信号 */
extern u16 Rc_Pwm_In[8];

/* PWM通道对应的映射,根据遥控器设置 */
s8 CH_in_Mapping[8] = {0,1,2,3,4,5,6,7};
/* PWM最大控制量,根据遥控器设置 */
s16 MAX_CH[8]  = {1900 ,1900 ,1900 ,1900 ,1900 ,1900 ,1900 ,1900 };
/* PWM最小控制量,根据遥控器设置 */
s16 MIN_CH[8]  = {1100 ,1100 ,1100 ,1100 ,1100 ,1100 ,1100 ,1100 };
/* PWM方向控制量,根据遥控器设置 */
char CH_DIR[8] = {0    ,0    ,0    ,0    ,0    ,0    ,0    ,0    };
/* 无控制信号缺省控制量状态 */
u16 STOP_CH[8] = {1500 ,1500 ,1000 ,1500 ,1500 ,1500 ,1000 ,1000 };

/* PWM配对过来的控制数据，范围1000-2000 */
u16 Mapped_CH[8];
/* 控制信号数据，范围是-500到+500 */
s16 CH[8];
/* 滤波后的信号数据，范围是-500到+500 */
float CH_filter[8];

/* 遥控器接收机控制方式，0是无PWM信号 1是有PWM信号 */
u8 RadioControl_Sign;
/* 接收到错误信号标识 */
u8 CH_Error[8];
/* PWM虚拟看门狗，监测通道信号错误 */
u16 RadioControl_Sign_cnt,CLR_CH_Error[8];

/* 数传控制方式，0是无数传控制 1是有数传控制 */
u8 DataControl_Sign;
/* 数传虚拟看门狗，监测通道信号错误 */
u16 DataControl_Sign_cnt;

/* 解锁判断标志
0未解锁 1已经解锁 */
u8 unlocked_to_fly = 0;
/* 等待解锁计时 */
s16 unlocked_to_fly_cnt=0;
/* 定高模式切换
0自稳 1气压定高 2超声定高 */
u8 height_ctrl_mode = 0;
/* 定点模式切换
1不定点 2记录返航点 3定点 */
u8 position_ctrl_mode = 1;
/* 标记上次控制模式 */
u8 position_ctrl_mode_old = 1;
/* 超声波有连接 */
extern u8 ultra_ok;

/*----------------------------------------------------------
 + 实现功能：通道对应的映射
 + 调用参数：输入数组,输出数组
----------------------------------------------------------*/
void CH_Mapping_Fun(u16 *in,u16 *Mapped_CH)
{
    for(u8 i = 0 ; i < 8 ; i++ )
    {
        /* 数组通道的映射 */
        *( Mapped_CH + i ) = *( in + CH_in_Mapping[i] );
    }
}

/*----------------------------------------------------------
 + 实现功能：解锁判断
 + 调用参数：两次调用时间差 单位：秒
----------------------------------------------------------*/
void unlocked_to_fly_judje(float difference_time)
{
    /* 油门控制信号小于10%时 */
    if( CH_filter[2] < -400 )
    {
        /* 外八解锁方式 */
#if(USE_TOE_IN_UNLOCK)
        /* 航向控制信号小于10%时 */
        if( CH_filter[3] < -400 )
        {
            /* 俯仰控制信号大于90%时 */
            if( CH_filter[1] > 400 )
            {
                /* 横滚控制信号小于10%时 */
                if( CH_filter[0] > 400 )
                {
                    /* 防止连续切换 */
                    if( unlocked_to_fly_cnt != -1 )
                    {
                        /* 解锁计时 */
                        unlocked_to_fly_cnt +=  1000 *T;
                    }
                }
            }
        }
#else
        /* 默认解锁方式 */
        /* 航向控制信号小于10%时 */
        if( CH_filter[3] < -400 )
        {
            /* 防止连续切换 且已经解锁 */
            if( unlocked_to_fly_cnt != -1 && unlocked_to_fly )
            {
                /* 解锁计时 */
                unlocked_to_fly_cnt += 1000 *difference_time;
            }
        }
        /* 航向控制信号大于90%时 */
        else if( CH_filter[3] > 400 )
        {
            /* 防止连续切换 且已经上锁 */
            if( unlocked_to_fly_cnt != -1 && !unlocked_to_fly )
            {
                /* 解锁计时 */
                unlocked_to_fly_cnt += 1000 *difference_time;
            }
        }
#endif
        /* 已经解锁或上锁了 */
        else if( unlocked_to_fly_cnt == -1 )						//4通道(CH[3])回位
        {
            /* 清零解锁计时 */
            unlocked_to_fly_cnt=0;
        }
    }
    /* 不符合解锁解锁或上锁条件 */
    else
    {
        /* 清零解锁计时 */
        unlocked_to_fly_cnt=0;
    }
    /* 清零数1000次以上 */
    if( unlocked_to_fly_cnt > 1000 )
    {
        /* 防止连续切换解锁 */
        unlocked_to_fly_cnt = -1;
        /* 若没有解锁 */
        if( !unlocked_to_fly )
            /* 解锁 */
            unlocked_to_fly = 1;
        /* 若已经解锁 */
        else
            /* 上锁 */
            unlocked_to_fly = 0;
    }
}

/*----------------------------------------------------------
 + 实现功能：由任务调度调用周期2ms
    接收机控制量数组：Mapped_CH 1000-1500-2000
    数传控制量数组：RX_CH     1000-1500-2000
    无控制量数组：STOP_CH   1000-1500-2000
----------------------------------------------------------*/
void Call_RadioContrl( float inner_loop_time )
{
    s16 CH_TMP[8];
    /* 积分滤波时间量 */
    float filter_A;

    /* 当接收到遥控器接收机的PWM控制时 */
    if( RadioControl_Sign == 1 )
        /* 要对通道顺序进行匹配 */
        CH_Mapping_Fun(Rc_Pwm_In,Mapped_CH);

    /* 8个输入通道独立配置 */
    for(u8 i = 0; i < 8 ; i++ )
    {
        /* 当接收到遥控器接收机的PWM控制时 */
        if( RadioControl_Sign == 1 )
        {
            /* 判断PWM数据越界错误 */
            if( (u16)Mapped_CH[i] > 2500 || (u16)Mapped_CH[i] < 500 )
            {
                /* 接收到错误信号标志 */
                CH_Error[i]=1;
                /* 清除错误信号计时 */
                CLR_CH_Error[i] = 0;//
            }
            /* PWM数据正常 */
            else
            {
                /* 清除错误信号计时并判断 */
                if( ++CLR_CH_Error[i] > 200 )
                {
                    /* 清除错误信号计时赋值 */
                    CLR_CH_Error[i] = 200;
                    /* 清除错误信号标志 */
                    CH_Error[i] = 0;
                }
            }

            /* 单PWM数据是正确的 */
            if(!CH_Error[i])
            {
                /* 映射拷贝数据，大约 1000~2000 */
                CH_TMP[i] = ( Mapped_CH[i] );

                /* 配对范围数组判断 */
                if( MAX_CH[i] > MIN_CH[i] )
                {
                    /* 方向零正向 */
                    if( !CH_DIR[i] )
                        CH[i] =   LIMIT ( (s16)( ( CH_TMP[i] - MIN_CH[i] )/(float)( MAX_CH[i] - MIN_CH[i] ) *1000 - 500 ), -500, 500);
                    /* 方向非零反向 */
                    else
                        CH[i] = - LIMIT ( (s16)( ( CH_TMP[i] - MIN_CH[i] )/(float)( MAX_CH[i] - MIN_CH[i] ) *1000 - 500 ), -500, 500);
                }
                /* 如果配对范围数组有错误 */
                else
                    /* PWM控制失能 */
                    RadioControl_Sign = 0;
            }
            /* 单通道PWM无数据或数据错误 */
            else
            {
                /* 当接收到数传控制的控制时 */
                if( DataControl_Sign == 1 )
                    /* 赋值并限幅 */
                    CH[i] = LIMIT ( (short)RX_CH[i] - 1500, -500, 500);
                /* 当没有接收到控制时 */
                else
                    /* 赋值 */
                    CH[i] = LIMIT ( (short)STOP_CH[i] - 1500, -500, 500);
            }
        }
        /* 当接收到数传控制的控制时 */
        else if( DataControl_Sign == 1 )
            /* 赋值并限幅 */
            CH[i] = LIMIT ( (short)RX_CH[i] - 1500, -500, 500);
        /* 当没有接收到控制时 */
        else
            /* 无控制信号缺省控制量状态 */
            CH[i] = LIMIT ( (short)STOP_CH[i] - 1500, -500, 500);

        /* 积分滤波时间量 */
        filter_A = 3.14f *20 *inner_loop_time;
        /* 信号连续变化正常,一般滤波幅度 */
        if( ABS(CH[i] - CH_filter[i]) <100 )
            CH_filter[i] += filter_A *(CH[i] - CH_filter[i]) ;
        /* 信号连续变化较大,低滤波幅度 */
        else
            CH_filter[i] += 0.5f *filter_A *( CH[i] - CH_filter[i]) ;
    }
}

/*----------------------------------------------------------
 + 实现功能：遥控信号虚拟看门狗，400ms内必须调用一次
 + 调用参数：控制信号类型：1为PWM信号 2为数传
----------------------------------------------------------*/
void Call_RadioControl_Sign(u8 sign)
{
    if(sign == 1)
    {
        /* 获取信号类型 */
        RadioControl_Sign = 1;
        /* 虚拟看门狗计时清零 */
        RadioControl_Sign_cnt = 0;
    }
    else if(sign == 2)
    {
        /* 获取信号类型 */
        DataControl_Sign = 1;
        /* 虚拟看门狗计时清零 */
        DataControl_Sign_cnt = 0;
    }
}

/*----------------------------------------------------------
 + 实现功能：控制模式判断 由任务调度调用周期50ms
----------------------------------------------------------*/
void Call_RadioControl_Mode()
{

    /* 遥控信号虚拟看门狗，判断是是否400ms无信号输入 */
    if(++RadioControl_Sign_cnt>8)
    {
        /* 虚拟看门狗计时清零 */
        RadioControl_Sign_cnt = 0;
        /* 标记为无信号输入 */
        RadioControl_Sign = 0;
    }

    /* 遥控信号虚拟看门狗，判断是是否400ms无信号输入 */
    if(++DataControl_Sign_cnt>8)
    {
        /* 虚拟看门狗计时清零 */
        DataControl_Sign_cnt = 0;
        /* 标记为无信号输入 */
        DataControl_Sign = 0;
    }

    /* 解锁判断 */
    unlocked_to_fly_judje(0.05f);

    /* 定高模式切换 由AUX1三档控制*/
    /* 辅助1通道在1000-1299 */
    if( CH_filter[AUX1] < -200 )
        /* ，非定高模式 */
        height_ctrl_mode = 0;
    /* 辅助1通道在1300-1699 */
    else if( CH_filter[AUX1] < 200 )
        /* 气压定高模式 */
        height_ctrl_mode = 1;
    /* 辅助1通道在1700-2000 */
    else
    {
        /* 超声波定高模式 */
        if(ultra_ok == 1)
            height_ctrl_mode = 2;
        /* 没有超声波就是气压定高模式 */
        else
            height_ctrl_mode = 1;
    }
	
	/* 位置模式切换 由AUX2三档控制*/
    /* 辅助2通道在1000-1299 */
    if( CH_filter[AUX2] > -200 && CH_filter[AUX2] < 200)
        /* 不悬停模式 */
        position_ctrl_mode = 1;
    /* 辅助2通道在1300-1699 */
    else if( CH_filter[AUX2] < -200 )
	{
        /* 航向锁定模式 */
        if(gpsx.fixmode >= 2)
            position_ctrl_mode = 2;
        /* 没有接收到GPS信号则切换为不悬停模式 */
        else
            position_ctrl_mode = 1;
	}
    /* 辅助2通道在1700-2000 */
    else
    {
        /* GPS定位模式 */
        if(gpsx.fixmode >= 2)
            position_ctrl_mode = 3;
        /* 没有接收到GPS信号则切换为不悬停模式 */
        else
            position_ctrl_mode = 1;
    }
}

