/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file device_iic.c
*@version V1.0
*@date  2018/5/24
*@brief IIC设备文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#include "device_iic.h"
#include "stm32f4xx.h"

#define SCL_H         GPIO_I2C->BSRRL = I2C_Pin_SCL
#define SCL_L         GPIO_I2C->BSRRH = I2C_Pin_SCL
#define SDA_H         GPIO_I2C->BSRRL = I2C_Pin_SDA
#define SDA_L         GPIO_I2C->BSRRH = I2C_Pin_SDA
#define SCL_read      GPIO_I2C->IDR  & I2C_Pin_SCL
#define SDA_read      GPIO_I2C->IDR  & I2C_Pin_SDA
#define GPIO_I2C		GPIOB
#define I2C_Pin_SCL		GPIO_Pin_6
#define I2C_Pin_SDA		GPIO_Pin_7
#define RCC_I2C		RCC_AHB1Periph_GPIOB

/* IIC延时长短 */
volatile u8 I2C_FastMode;

/*----------------------------------------------------------
 + 实现功能：IIC设备延时
----------------------------------------------------------*/
void I2c_Soft_delay()
{
    u8 i = 10;
    while(i--);

    if(!I2C_FastMode)
    {
        u8 i = 15;
        while(i--);
    }
}

/*----------------------------------------------------------
 + 实现功能：IIC设备初始化
----------------------------------------------------------*/
void I2c_Device_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_I2C , ENABLE );
    GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SCL | I2C_Pin_SDA;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIO_I2C, &GPIO_InitStructure);
}

/*----------------------------------------------------------
 + 实现功能：IIC发送Start
 + 返回参数功能：0忙 1空闲
----------------------------------------------------------*/
int I2c_Soft_Start()
{
    SDA_H;
    SCL_H;
    I2c_Soft_delay();
    /* SDA线为低电平则总线忙 */
    if(!SDA_read)return 0;
    SDA_L;
    I2c_Soft_delay();
    /* SDA线为高电平则总线出错 */
    if(SDA_read) return 0;
    SDA_L;
    I2c_Soft_delay();
    return 1;
}

/*----------------------------------------------------------
 + 实现功能：IIC发送Stop
----------------------------------------------------------*/
void I2c_Soft_Stop()
{
    SCL_L;
    I2c_Soft_delay();
    SDA_L;
    I2c_Soft_delay();
    SCL_H;
    I2c_Soft_delay();
    SDA_H;
    I2c_Soft_delay();
}

/*----------------------------------------------------------
 + 实现功能：IIC发送Ask
----------------------------------------------------------*/
void I2c_Soft_Ask()
{
    SCL_L;
    I2c_Soft_delay();
    SDA_L;
    I2c_Soft_delay();
    SCL_H;
    I2c_Soft_delay();
    SCL_L;
    I2c_Soft_delay();
}

/*----------------------------------------------------------
 + 实现功能：IIC发送NoAsk
----------------------------------------------------------*/
void I2c_Soft_NoAsk()
{
    SCL_L;
    I2c_Soft_delay();
    SDA_H;
    I2c_Soft_delay();
    SCL_H;
    I2c_Soft_delay();
    SCL_L;
    I2c_Soft_delay();
}

/*----------------------------------------------------------
 + 实现功能：IIC等待ASK数据
 + 返回参数功能：0有ASK 1无ASK
----------------------------------------------------------*/
int I2c_Soft_WaitAsk(void)
{
    u8 ErrTime = 0;
    SCL_L;
    I2c_Soft_delay();
    SDA_H;
    I2c_Soft_delay();
    SCL_H;
    I2c_Soft_delay();
    while(SDA_read)
    {
        ErrTime++;
        if(ErrTime>50)
        {
            I2c_Soft_Stop();
            return 1;
        }
    }
    SCL_L;
    I2c_Soft_delay();
    return 0;
}

/*----------------------------------------------------------
 + 实现功能：IIC发送单字节数据
 + 调用参数功能：发送的数据从高位到低位
----------------------------------------------------------*/
void I2c_Soft_SendByte(u8 SendByte) //数据从高位到低位//
{
    u8 i=8;
    while(i--)
    {
        SCL_L;
        I2c_Soft_delay();
        if(SendByte&0x80)
            SDA_H;
        else
            SDA_L;
        SendByte<<=1;
        I2c_Soft_delay();
        SCL_H;
        I2c_Soft_delay();
    }
    SCL_L;
}

/*----------------------------------------------------------
 + 实现功能：IIC读取单字节数据
 + 调用参数功能：ack=1时，发送ACK，ack=0，发送NACK
 + 返回参数功能：接收到的数据从高位到低位
----------------------------------------------------------*/
u8 I2c_Soft_ReadByte(u8 ask)
{
    u8 i=8;
    u8 ReceiveByte=0;

    SDA_H;
    while(i--)
    {
        ReceiveByte<<=1;
        SCL_L;
        I2c_Soft_delay();
        SCL_H;
        I2c_Soft_delay();
        if(SDA_read)
        {
            ReceiveByte|=0x01;
        }
    }
    SCL_L;

    if (ask)
        I2c_Soft_Ask();
    else
        I2c_Soft_NoAsk();
    return ReceiveByte;
}

/*----------------------------------------------------------
 + 实现功能：IIC写入单字节数据
 + 调用参数功能：设备，寄存器，数据
 + 返回参数功能：0正常 1异常
----------------------------------------------------------*/
u8 IIC_Write_1Byte(u8 SlaveAddress,u8 REG_Address,u8 REG_data)
{
    I2c_Soft_Start();
    I2c_Soft_SendByte(SlaveAddress<<1);
    if(I2c_Soft_WaitAsk())
    {
        I2c_Soft_Stop();
        return 1;
    }
    I2c_Soft_SendByte(REG_Address);
    I2c_Soft_WaitAsk();
    I2c_Soft_SendByte(REG_data);
    I2c_Soft_WaitAsk();
    I2c_Soft_Stop();
    return 0;
}

/*----------------------------------------------------------
 + 实现功能：IIC读取单字节数据
 + 调用参数功能：设备，寄存器，数据
 + 返回参数功能：0正常 1异常
----------------------------------------------------------*/
u8 IIC_Read_1Byte(u8 SlaveAddress,u8 REG_Address,u8 *REG_data)
{
    I2c_Soft_Start();
    I2c_Soft_SendByte(SlaveAddress<<1);
    if(I2c_Soft_WaitAsk())
    {
        I2c_Soft_Stop();
        return 1;
    }
    I2c_Soft_SendByte(REG_Address);
    I2c_Soft_WaitAsk();
    I2c_Soft_Start();
    I2c_Soft_SendByte(SlaveAddress<<1 | 0x01);
    I2c_Soft_WaitAsk();
    *REG_data= I2c_Soft_ReadByte(0);
    I2c_Soft_Stop();
    return 0;
}

/*----------------------------------------------------------
 + 实现功能：IIC写入多字节数据
 + 调用参数功能：设备，寄存器，数据长度，数据
 + 返回参数功能：0正常 1异常
----------------------------------------------------------*/
u8 IIC_Write_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf)
{
    I2c_Soft_Start();
    I2c_Soft_SendByte(SlaveAddress<<1);
    if(I2c_Soft_WaitAsk())
    {
        I2c_Soft_Stop();
        return 1;
    }
    I2c_Soft_SendByte(REG_Address);
    I2c_Soft_WaitAsk();
    while(len--)
    {
        I2c_Soft_SendByte(*buf++);
        I2c_Soft_WaitAsk();
    }
    I2c_Soft_Stop();
    return 0;
}

/*----------------------------------------------------------
 + 实现功能：IIC读取多字节数据
 + 调用参数功能：设备，寄存器，数据长度，数据
 + 返回参数功能：0正常 1异常
----------------------------------------------------------*/
u8 IIC_Read_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf)
{
    I2c_Soft_Start();
    I2c_Soft_SendByte(SlaveAddress<<1);
    if(I2c_Soft_WaitAsk())
    {
        I2c_Soft_Stop();
        return 1;
    }
    I2c_Soft_SendByte(REG_Address);
    I2c_Soft_WaitAsk();

    I2c_Soft_Start();
    I2c_Soft_SendByte(SlaveAddress<<1 | 0x01);
    I2c_Soft_WaitAsk();
    while(len)
    {
        if(len == 1)
        {
            *buf = I2c_Soft_ReadByte(0);
        }
        else
        {
            *buf = I2c_Soft_ReadByte(1);
        }
        buf++;
        len--;
    }
    I2c_Soft_Stop();
    return 0;
}

