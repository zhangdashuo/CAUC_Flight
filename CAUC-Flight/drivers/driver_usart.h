/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.
 + 文件名  ：driver_usart.h
 + 描述    ：串口驱动头文件
代码尽量做到逐行注释 代码有问题，及时加群交流 作者有偿支持对开源代码的完善 */
#ifndef _DRIVER_USART_H
#define _DRIVER_USART_H

#include "stm32f4xx.h"
#include "device_usart.h"

/*----------------------------------------------------------
 + 实现功能：串口2发送数据调用
 + 调用参数功能：字符串数组：发送数据，单字节整数：数据长度
----------------------------------------------------------*/
extern void Usart2_Send(unsigned char *DataToSend ,u8 data_num);

/*----------------------------------------------------------
 + 实现功能：串口4发送数据调用
 + 调用参数功能：字符串数组：发送数据，单字节整数：数据长度
----------------------------------------------------------*/
extern void Uart4_Send(unsigned char *DataToSend ,u8 data_num);

/*----------------------------------------------------------
 + 实现功能：串口5发送数据调用
 + 调用参数功能：字符串数组：发送数据，单字节整数：数据长度
----------------------------------------------------------*/
extern void Uart5_Send(unsigned char *DataToSend ,u8 data_num);

#endif
/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
