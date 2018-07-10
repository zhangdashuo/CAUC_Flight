/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file driver_ultrasonic.c
*@version V1.0
*@date  2018/5/24
*@brief 超声波测距驱动文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#include "stm32f4xx.h"
#include "driver_ultrasonic.h"
#include "driver_usart.h"
#include "database.h"

/* 超声波模块型号选择宏定义，可选US100型号或者KS103 */
#define USE_US100   //#define USE_KS103

/* 超声波模块状态标志位：1准备接收高位 2准备接受低位 */
s8 ultra_state;
/* 检测是否连接超声波模块 */
u8 ultra_ok = 0;
/* 超声波模块测量距离：单位毫米 */
u16 ultra_distance,ultra_distance_old;
/* 超声波模块测量距离两次差：单位毫米 */
s16 ultra_delta;

/*----------------------------------------------------------
 + 实现功能：超声波模块初始化
----------------------------------------------------------*/
void Ultrasonic_Init()
{
    /* 串口5初始化，波特率9600 */
    Device_Usart5_ENABLE_Init(9600,2,1,DISABLE,ENABLE);
}

/*----------------------------------------------------------
 + 实现功能：由任务调度调用周期100ms
----------------------------------------------------------*/
void Call_Ultrasonic()
{
    /* 选择了KS103超声波模块 */
#if defined(USE_KS103)
    /* KS103超声波模块默认配置数据 */
    u8 temp[3] = {0xe8,0x02,0xbc};
    /* 发送配置数据 */
    Uart5_Send(temp ,3);

    /* 选择了US100超声波模块 */
#elif defined(USE_US100)
    /* US100超声波模块默认配置数据 */
    u8 temp = 0x55;
    /* 发送配置数据 */
    Uart5_Send(&temp ,1);

    /* 结束选择模块宏定义 */
#endif

    /* 开始接收高位数据 */
    ultra_state = 1;
}

/*----------------------------------------------------------
 + 实现功能：串口5接收到数据解析
 + 调用参数功能：单字节整数：串口接收到的数据
----------------------------------------------------------*/
void Ultra_Get(u8 com_data)
{
    /* 接收数据暂存 */
    static u8 ultra_tmp;

    /*判断是高位数据*/
    if( ultra_state == 1 )
    {
        /* 记录下数据 */
        ultra_tmp = com_data;
        /* 下次判断低位数据 */
        ultra_state = 2;
    }

    /*判断是低位数据*/
    else if( ultra_state == 2 )
    {
        /* 接收测量数据，单位毫米 */
        ultra_distance = (ultra_tmp<<8) + com_data;
        /* 置为状态位 */
        ultra_state = 0;
        /* 正确获取了数据 */
        ultra_ok = 1;
    }

    /* 最近两次测距差 */
    ultra_delta = ultra_distance - ultra_distance_old;
    /* 存储测距数据 */
    ultra_distance_old = ultra_distance;
}

