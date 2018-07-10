/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file main.c
*@version V1.0
*@date  2018/5/24
*@brief 数据传输头文件
*@design 无人机研究小组
	     谭政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#ifndef _DATA_TRANSFER_H
#define	_DATA_TRANSFER_H

#include "stm32f4xx.h"
#include "height_ctrl.h"

//#define DT_USE_USART2 			//开启串口2数传功能
#define DT_USE_USB_HID		//开启飞控USBHID连接上位机功能
#define DT_USE_ATO_SP			//使用匿名发送速度
//#define DT_USE_LIT_SP			//使用light发送速度

/* 等待发送数据的标志 */
extern u8 wait_for_translate;
/* 等待发送数据的标志结构体 */
typedef struct
{
    u8 send_status;
    u8 send_speed;
    u8 send_rcdata;
    u8 send_motopwm;
    u8 send_senser;
    u8 send_senser2;
    u8 send_location;
    u8 send_power;
    u8 send_pid1;
    u8 send_pid2;
    u8 send_pid3;
    u8 send_pid4;
    u8 send_user;
} dt_flag_t;

/*----------------------------------------------------------
 + 实现功能：由任务调度调用周期1ms
----------------------------------------------------------*/
extern void Call_Data_transfer(void);

/*----------------------------------------------------------
 + 实现功能：数传初始化
----------------------------------------------------------*/
extern void Data_transfer_init(void);

/*----------------------------------------------------------
 + 实现功能：数据接收并保存
 + 调用参数：接收到的单字节数据
----------------------------------------------------------*/
extern void DT_Data_Receive_Prepare(u8 data);

#endif

