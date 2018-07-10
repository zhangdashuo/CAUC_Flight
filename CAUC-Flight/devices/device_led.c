/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file device_led.c
*@version V1.0
*@date  2018/5/24
*@brief led设备文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#include "device_led.h"
#include "stm32f4xx.h"

/*----------------------------------------------------------
 + 实现功能：控制LED初始化
----------------------------------------------------------*/
void LED_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_LED,ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_Pin   = Pin_LED1| Pin_LED2| Pin_LED3| Pin_LED4;
    GPIO_Init(GPIO_LED, &GPIO_InitStructure);

    GPIO_SetBits(GPIO_LED, Pin_LED1);
    GPIO_SetBits(GPIO_LED, Pin_LED2);
    GPIO_SetBits(GPIO_LED, Pin_LED3);
    GPIO_SetBits(GPIO_LED, Pin_LED4);
}

/*----------------------------------------------------------
 + 实现功能：控制LED发光亮度 由任务调度调用周期1ms
 + 调用参数功能：发光亮度数组 0-20
----------------------------------------------------------*/
void Call_LED_show( u8 duty[4] )
{
    /* 计时值 */
    static u8 LED_cnt[4];

    /* 依次控制4个LED */
    for(u8 i=0; i<4; i++)
    {
        /* 计时器用于比较 */
        if( LED_cnt[i] < 19 )
            LED_cnt[i]++;
        else
            LED_cnt[i] = 0;

        /* LED开启状态控制 */
        if( LED_cnt[i] < duty[i] )
        {
            /* 依次控制4个LED */
            switch(i)
            {
            case 0:
                LED1_ON;
                break;
            case 1:
                LED2_ON;
                break;
            case 2:
                LED3_ON;
                break;
            case 3:
                LED4_ON;
                break;
            }
        }
        /* LED关断状态控制 */
        else
        {
            /* 依次控制4个LED */
            switch(i)
            {
            case 0:
                LED1_OFF;
                break;
            case 1:
                LED2_OFF;
                break;
            case 2:
                LED3_OFF;
                break;
            case 3:
                LED4_OFF;
                break;
            }
        }
    }
}

