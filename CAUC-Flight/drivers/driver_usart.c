/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file driver_usart.c
*@version V1.0
*@date  2018/5/24
*@brief 串口驱动文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#include "stm32f4xx.h"
#include "driver_usart.h"
#include "data_transfer.h"
#include "Optical_Flow.h"
#include "driver_ultrasonic.h"
#include "driver_GPS.h"

/* 需要发送的数据的数组 */
u8 USART2_TxBuffer[256];
/* 正在发送的数据的数组下标 */
u8 USART2_TxCounter=0;
/* 需要发送的数据的总长度 */
u8 USART2_count=0;

/* 需要发送的数据的数组 */
u8 USART3_TxBuffer[32];
/* 正在发送的数据的数组下标 */
u8 USART3_TxCounter=0;
/* 需要发送的数据的总长度 */
u8 USART3_count=0;

/* 需要发送的数据的数组 */
u8 UART4_TxBuffer[256];
/* 正在发送的数据的数组下标 */
u8 UART4_TxCounter=0;
/* 需要发送的数据的总长度 */
u8 UART4_count=0;

/* 需要发送的数据的数组 */
u8 UART5_TxBuffer[256];
/* 正在发送的数据的数组下标 */
u8 UART5_TxCounter=0;
/* 需要发送的数据的总长度 */
u8 UART5_count=0;

/*----------------------------------------------------------
 + 实现功能：串口2发送数据
 + 保留函数名，由中断调用
----------------------------------------------------------*/
void USART2_IRQHandler(void)
{
    /* 接收数据临时变量 */
    u8 com_data;

    /* 判断过载错误中断 */
    if(USART2->SR & USART_SR_ORE)
        com_data = USART2->DR;

    /* 判断是否接收中断 */
    if( USART_GetITStatus(USART2,USART_IT_RXNE) )
    {
        /* 清除中断标志 */
        USART_ClearITPendingBit(USART2,USART_IT_RXNE);

        /* 接收数据及后续的任务 */
        com_data = USART2->DR;

        /* 数传数据处理解析 */
        DT_Data_Receive_Prepare(com_data);
    }

    /* 判断是否发送中断 */
    if( USART_GetITStatus(USART2,USART_IT_TXE ) )
    {
        /* 赋值需要发送的数据，数组下标进位 */
        USART2->DR = USART2_TxBuffer[USART2_TxCounter++];

        /* 判断数组长度条件，关闭发送中断，退出发送状态 */
        if(USART2_TxCounter == USART2_count)
            USART2->CR1 &= ~USART_CR1_TXEIE;
    }
}

/*----------------------------------------------------------
 + 实现功能：串口3发送数据
 + 保留函数名，由中断调用
----------------------------------------------------------*/
void USART3_IRQHandler(void)
{
    /* 接收数据临时变量 */
    u8 com_data;

    /* 判断过载错误中断 */
    if(USART3->SR & USART_SR_ORE)
        com_data = USART3->DR;

    /* 判断是否接收中断 */
    if( USART_GetITStatus(USART3,USART_IT_RXNE) )
    {
        /* 清除中断标志 */
        USART_ClearITPendingBit(USART3,USART_IT_RXNE);

        /* 接收数据及后续的任务 */
        com_data = USART3->DR;

        /* 光流数据处理解析 */
        Optical_Flow_Receive_Prepare(com_data);
    }

    /* 判断是否发送中断 */
    if( USART_GetITStatus(USART3,USART_IT_TXE ) )
    {
        /* 赋值需要发送的数据，数组下标进位 */
        USART3->DR = USART3_TxBuffer[USART3_TxCounter++];

        /* 判断数组长度条件，关闭发送中断，退出发送状态 */
        if(USART3_TxCounter == USART3_count)
            USART3->CR1 &= ~USART_CR1_TXEIE;
    }
}

/*----------------------------------------------------------
 + 实现功能：串口4发送数据
 + 保留函数名，由中断调用
----------------------------------------------------------*/
void UART4_IRQHandler(void)
{
    /* 接收数据临时变量 */
    u8 com_data;

    /* 判断过载错误中断 */
    if(UART4->SR & USART_SR_ORE)
        com_data = UART4->DR;

    /* 判断是否接收中断 */
    if( USART_GetITStatus(UART4,USART_IT_RXNE) )
    {
        /* 清除中断标志 */
        USART_ClearITPendingBit(UART4,USART_IT_RXNE);

        /* 接收数据及后续的任务 */
        com_data = UART4->DR;

        /* GPS数据处理解析 */
        GPS_Get(com_data);
		/* 给GPS喂狗 */
		GPS_DOG = 0;
    }

    /* 判断是否发送中断 */
    if( USART_GetITStatus(UART4,USART_IT_TXE ) )
    {
        /* 赋值需要发送的数据，数组下标进位 */
        UART4->DR = UART4_TxBuffer[UART4_TxCounter++];

        /* 判断数组长度条件，关闭发送中断，退出发送状态 */
        if(UART4_TxCounter >= UART4_count)
            UART4->CR1 &= ~USART_CR1_TXEIE;
    }
}

/*----------------------------------------------------------
 + 实现功能：串口5发送数据
 + 保留函数名，由中断调用
----------------------------------------------------------*/
void UART5_IRQHandler(void)
{
    /* 接收数据临时变量 */
    u8 com_data;

    /* 判断过载错误中断 */
    if(UART5->SR & USART_SR_ORE)
        com_data = UART5->DR;

    /* 判断是否接收中断 */
    if( USART_GetITStatus(UART5,USART_IT_RXNE) )
    {
        /* 清除中断标志 */
        USART_ClearITPendingBit(UART5,USART_IT_RXNE);

        /* 接收数据及后续的任务 */
        com_data = UART5->DR;

        /* 超声波数据处理解析 */
        Ultra_Get(com_data);
    }

    /* 判断是否发送中断 */
    if( USART_GetITStatus(UART5,USART_IT_TXE ) )
    {
        /* 赋值需要发送的数据，数组下标进位 */
        UART5->DR = UART5_TxBuffer[UART5_TxCounter++];

        /* 判断数组长度条件，关闭发送中断，退出发送状态 */
        if(UART5_TxCounter >= UART5_count)
            UART5->CR1 &= ~USART_CR1_TXEIE;
    }
}

/*----------------------------------------------------------
 + 实现功能：串口2发送数据调用
 + 调用参数功能：字符串数组：发送数据，单字节整数：数据长度
----------------------------------------------------------*/
void Usart2_Send(unsigned char *DataToSend ,u8 data_num)
{
    /* data_num用于控制循环次数，将DataToSend内容赋值到UART2_TxBuffer数组 */
    for(u8 i=0; i<data_num; i++)
        USART2_TxBuffer[USART2_count++] = *(DataToSend+i);

    /* 判断 发送缓冲区中断使能 为非，打开发送中断，进入发送状态 */
    if(!(USART2->CR1 & USART_CR1_TXEIE))
        USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
}

/*----------------------------------------------------------
 + 实现功能：串口4发送数据调用
 + 调用参数功能：字符串数组：发送数据，单字节整数：数据长度
----------------------------------------------------------*/
void Uart4_Send(unsigned char *DataToSend ,u8 data_num)
{
    /* data_num用于控制循环次数，将DataToSend内容赋值到UART4_TxBuffer数组 */
    for(u8 i=0; i<data_num; i++)
        UART4_TxBuffer[UART4_count++] = *(DataToSend+i);

    /* 判断 发送缓冲区中断使能 为非，打开发送中断，进入发送状态 */
    if(!(UART4->CR1 & USART_CR1_TXEIE))
        USART_ITConfig(UART4, USART_IT_TXE, ENABLE);
}

/*----------------------------------------------------------
 + 实现功能：串口5发送数据调用
 + 调用参数功能：字符串数组：发送数据，单字节整数：数据长度
----------------------------------------------------------*/
void Uart5_Send(unsigned char *DataToSend ,u8 data_num)
{
    /* data_num用于控制循环次数，将DataToSend内容赋值到UART5_TxBuffer数组 */
    for(u8 i=0; i<data_num; i++)
        UART5_TxBuffer[UART5_count++] = *(DataToSend+i);

    /* 判断 发送缓冲区中断使能 为非，打开发送中断，进入发送状态 */
    if(!(UART5->CR1 & USART_CR1_TXEIE))
        USART_ITConfig(UART5, USART_IT_TXE, ENABLE);
}

