/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file main.c
*@version V1.0
*@date  2018/5/24
*@brief GPS数据解析处理头文件
*@design 无人机研究小组
	     谭政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#ifndef _DRIVER_GPS_H
#define _DRIVER_GPS_H

#include "stm32f4xx.h"
#include "driver_usart.h"

/* GPS使用串口 */
#define GPSUSART 			UART4
/* GPS发送函数 */
#define Ublox_Send_Date(dbuf,len) Uart4_Send(dbuf,len)
/* 最大接收缓存字节数 */
#define GPSUSART_MAX_RECV_LEN		800
/* 最大接收缓存字节数 */
#define GPSUSART_RX_BUF_old_LEN     800
/* 最大发送缓存字节数 */
#define GPSUSART_MAX_SEND_LEN		300

/* 接收缓冲,最大GPSUSART_MAX_RECV_LEN字节 */
extern u8  GPSUSART_RX_BUF[GPSUSART_MAX_RECV_LEN];
/* 接收缓冲,最大GPSUSART_RX_BUF_old_LEN字节 */
extern u8  GPSUSART_RX_BUF_old[GPSUSART_RX_BUF_old_LEN];
/* 发送缓冲,最大GPSUSART_MAX_SEND_LEN字节 */
extern u8  GPSUSART_TX_BUF[GPSUSART_MAX_SEND_LEN];
/*接收数据状态 */
extern vu16 GPSUSART_RX_STA;
/* GPS看门狗 */
extern u8 GPS_DOG;

/* GPS NMEA-0183协议重要参数结构体定义 */
/* 卫星信息 */
typedef struct
{
    u8 num;		//卫星编号
    u8 eledeg;	//卫星仰角
    u16 azideg;	//卫星方位角
    u8 sn;		//信噪比
} nmea_slmsg;

/* UTC时间信息 */
typedef struct
{
    u16 year;	//年份
    u8 month;	//月份
    u8 date;	//日期
    u8 hour; 	//小时
    u8 min; 	//分钟
    u8 sec; 	//秒钟
} nmea_utc_time;

/* NMEA 0183 协议解析后数据存放结构体 */
typedef struct
{
    u8 svnum;					//可见卫星数
    nmea_slmsg slmsg[12];		//最多12颗卫星
    nmea_utc_time utc;			//UTC时间
    u32 latitude;				//纬度 分扩大100000倍,实际要除以100000
    u8 nshemi;					//北纬/南纬,N:北纬;S:南纬
    u32 longitude;			    //经度 分扩大100000倍,实际要除以100000
    u8 ewhemi;					//东经/西经,E:东经;W:西经
    u8 gpssta;					//GPS状态:0,未定位;1,非差分定位;2,差分定位;6,正在估算.
    u8 posslnum;				//用于定位的卫星数,0~12.
    u8 possl[12];				//用于定位的卫星编号
    u8 fixmode;					//定位类型:1,没有定位;2,2D定位;3,3D定位
    u16 pdop;					//位置精度因子 0~500,对应实际值0~50.0
    u16 hdop;					//水平精度因子 0~500,对应实际值0~50.0
    u16 vdop;					//垂直精度因子 0~500,对应实际值0~50.0
    int altitude;			 	//海拔高度,放大了10倍,实际除以10.单位:0.1m
    u16 speed;					//地面速率,放大了1000倍,实际除以10.单位:0.001公里/小时
} nmea_msg;

/* UBLOX GPS 配置(清除,保存,加载等)结构体 */
typedef struct
{
    u16 header;					//cfg header,固定为0X62B5(小端模式)
    u16 id;						//CFG CFG ID:0X0906 (小端模式)
    u16 dlength;				//数据长度 12/13
    u32 clearmask;				//子区域清除掩码(1有效)
    u32 savemask;				//子区域保存掩码
    u32 loadmask;				//子区域加载掩码
    u8  devicemask; 		  	//目标器件选择掩码	b0:BK RAM;b1:FLASH;b2,EEPROM;b4,SPI FLASH
    u8  cka;		 			//校验CK_A
    u8  ckb;			 		//校验CK_B
} _ublox_cfg_cfg;

/* UBLOX GPS 消息设置结构体 */
typedef struct
{
    u16 header;					//cfg header,固定为0X62B5(小端模式)
    u16 id;						//CFG MSG ID:0X0106 (小端模式)
    u16 dlength;				//数据长度 8
    u8  msgclass;				//消息类型(F0 代表NMEA消息格式)
    u8  msgid;					//消息 ID
    //00,GPGGA;01,GPGLL;02,GPGSA;
    //03,GPGSV;04,GPRMC;05,GPVTG;
    //06,GPGRS;07,GPGST;08,GPZDA;
    //09,GPGBS;0A,GPDTM;0D,GPGNS;
    u8  iicset;					//IIC消输出设置    0,关闭;1,使能.
    u8  uart1set;				//UART1输出设置	   0,关闭;1,使能.
    u8  uart2set;				//UART2输出设置	   0,关闭;1,使能.
    u8  usbset;					//USB输出设置	   0,关闭;1,使能.
    u8  spiset;					//SPI输出设置	   0,关闭;1,使能.
    u8  ncset;					//未知输出设置	   默认为1即可.
    u8  cka;			 		//校验CK_A
    u8  ckb;			    	//校验CK_B
} _ublox_cfg_msg;

/* UBLOX GPS UART端口设置结构体 */
typedef struct
{
    u16 header;					//cfg header,固定为0X62B5(小端模式)
    u16 id;						//CFG PRT ID:0X0006 (小端模式)
    u16 dlength;				//数据长度 20
    u8  portid;					//端口号,0=IIC;1=UART1;2=UART2;3=USB;4=SPI;
    u8  reserved;				//保留,设置为0
    u16 txready;				//TX Ready引脚设置,默认为0
    u32 mode;					//串口工作模式设置,奇偶校验,停止位,字节长度等的设置.
    u32 baudrate;				//波特率设置
    u16 inprotomask;		 	//输入协议激活屏蔽位  默认设置为0X07 0X00即可.
    u16 outprotomask;		 	//输出协议激活屏蔽位  默认设置为0X07 0X00即可.
    u16 reserved4; 				//保留,设置为0
    u16 reserved5; 				//保留,设置为0
    u8  cka;			 		//校验CK_A
    u8  ckb;			    	//校验CK_B
} _ublox_cfg_prt;

/* UBLOX GPS 时钟脉冲配置结构体 */
typedef struct
{
    u16 header;					//cfg header,固定为0X62B5(小端模式)
    u16 id;						//CFG TP ID:0X0706 (小端模式)
    u16 dlength;				//数据长度
    u32 interval;				//时钟脉冲间隔,单位为us
    u32 length;				 	//脉冲宽度,单位为us
    signed char status;			//时钟脉冲配置:1,高电平有效;0,关闭;-1,低电平有效.
    u8 timeref;			   		//参考时间:0,UTC时间;1,GPS时间;2,当地时间.
    u8 flags;					//时间脉冲设置标志
    u8 reserved;				//保留
    signed short antdelay;	 	//天线延时
    signed short rfdelay;		//RF延时
    signed int userdelay; 	 	//用户延时
    u8 cka;						//校验CK_A
    u8 ckb;						//校验CK_B
} _ublox_cfg_tp;

/* UBLOX GPS 刷新速率配置结构体 */
typedef struct
{
    u16 header;					//cfg header,固定为0X62B5(小端模式)
    u16 id;						//CFG RATE ID:0X0806 (小端模式)
    u16 dlength;				//数据长度
    u16 measrate;				//测量时间间隔，单位为ms，最少不能小于200ms（5Hz）
    u16 navrate;				//导航速率（周期），固定为1
    u16 timeref;				//参考时间：0=UTC Time；1=GPS Time；
    u8  cka;					//校验CK_A
    u8  ckb;					//校验CK_B
} _ublox_cfg_rate;

/* NMEA 0183 协议解析后数据存放结构体 */
extern nmea_msg gpsx;

/*----------------------------------------------------------
 + 实现功能：GPS模块初始化
----------------------------------------------------------*/
extern void GPS_Init(void);

/*----------------------------------------------------------
 + 实现功能：由任务调度调用周期100ms
----------------------------------------------------------*/
extern void Call_GPS(void);

/*----------------------------------------------------------
 + 实现功能：由串口接收中断调用
----------------------------------------------------------*/
extern void GPS_Get(u8);

#endif

