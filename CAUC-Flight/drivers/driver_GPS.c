/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file driver_GPS.c
*@version V1.0
*@date  2018/5/24
*@brief GPS数据解析处理文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#include "driver_GPS.h"
#include "device_usart.h"
#include "time.h"
#include "stm32f4xx.h"
#include "stdarg.h"
#include "string.h"

/* 接收缓冲,最大GPSUSART_MAX_RECV_LEN字节 */
u8  GPSUSART_RX_BUF[GPSUSART_MAX_RECV_LEN];
/* 接收缓冲,最大GPSUSART_RX_BUF_old_LEN字节 */
u8  GPSUSART_RX_BUF_old[GPSUSART_RX_BUF_old_LEN];
/* 发送缓冲,最大GPSUSART_MAX_SEND_LEN字节 */
u8  GPSUSART_TX_BUF[GPSUSART_MAX_SEND_LEN];
/*接收数据状态 */
vu16 GPSUSART_RX_STA=0;
/* GPS看门狗 */
u8 GPS_DOG = 0;

/*----------------------------------------------------------
 + 调用参数功能：从buf里面得到第cx个逗号所在的位置
 + 返回值:0~0XFE,代表逗号所在位置的偏移,0XFF,代表不存在第cx个逗号
----------------------------------------------------------*/
u8 NMEA_Comma_Pos(u8 *buf,u8 cx)
{
    u8 *p=buf;
    while(cx)
    {
        /* 遇到'*'或者非法字符,则不存在第cx个逗号 */
        if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;
        if(*buf==',')cx--;
        buf++;
    }
    return buf-p;
}

/*----------------------------------------------------------
 + 调用参数功能：m^n函数
 + 返回值::m^n次方
----------------------------------------------------------*/
u32 NMEA_Pow(u8 m,u8 n)
{
    u32 result=1;
    while(n--)result*=m;
    return result;
}

/*----------------------------------------------------------
 + 实现功能：将以','或者'*'结束的字符串转换为数字,
 + 调用参数功能：
 - buf:数字存储区
 - dx:小数点位数,返回给调用函数
 + 返回值:转换后的数值
----------------------------------------------------------*/
int NMEA_Str2num(u8 *buf,u8*dx)
{
    u8 *p=buf;
    u32 ires=0,fres=0;
    u8 ilen=0,flen=0,i;
    u8 mask=0;
    int res;
    while(1) //得到整数和小数的长度
    {
        if(*p=='-')
        {
            mask|=0X02;    //是负数
            p++;
        }
        if(*p==','||(*p=='*'))break;//遇到结束了
        if(*p=='.')
        {
            mask|=0X01;    //遇到小数点了
            p++;
        }
        else if(*p>'9'||(*p<'0'))	//有非法字符
        {
            ilen=0;
            flen=0;
            break;
        }
        if(mask&0X01)flen++;
        else ilen++;
        p++;
    }
    if(mask&0X02)buf++;	//去掉负号
    for(i=0; i<ilen; i++)	//得到整数部分数据
    {
        ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
    }
    if(flen>5)flen=5;	//最多取5位小数
    *dx=flen;	 		//小数点位数
    for(i=0; i<flen; i++)	//得到小数部分数据
    {
        fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
    }
    res=ires*NMEA_Pow(10,flen)+fres;
    if(mask&0X02)res=-res;
    return res;
}

/*----------------------------------------------------------
 + 实现功能：分析GPGSV信息
 + 调用参数功能：
 - gpsx:nmea信息结构体
 - buf:接收到的GPS数据缓冲区首地址
----------------------------------------------------------*/
void NMEA_GPGSV_Analysis(nmea_msg *gpsx,u8 *buf)
{
    u8 *p,*p1,dx;
    u8 len,i,j,slx=0;
    u8 posx;
    p=buf;
    p1=(u8*)strstr((const char *)p,"$GPGSV");
    len=p1[7]-'0';								//得到GPGSV的条数
    posx=NMEA_Comma_Pos(p1,3); 					//得到可见卫星总数
    if(posx!=0XFF)gpsx->svnum=NMEA_Str2num(p1+posx,&dx);
    for(i=0; i<len; i++)
    {
        p1=(u8*)strstr((const char *)p,"$GPGSV");
        for(j=0; j<4; j++)
        {
            posx=NMEA_Comma_Pos(p1,4+j*4);
            if(posx!=0XFF)gpsx->slmsg[slx].num=NMEA_Str2num(p1+posx,&dx);	//得到卫星编号
            else break;
            posx=NMEA_Comma_Pos(p1,5+j*4);
            if(posx!=0XFF)gpsx->slmsg[slx].eledeg=NMEA_Str2num(p1+posx,&dx);//得到卫星仰角
            else break;
            posx=NMEA_Comma_Pos(p1,6+j*4);
            if(posx!=0XFF)gpsx->slmsg[slx].azideg=NMEA_Str2num(p1+posx,&dx);//得到卫星方位角
            else break;
            posx=NMEA_Comma_Pos(p1,7+j*4);
            if(posx!=0XFF)gpsx->slmsg[slx].sn=NMEA_Str2num(p1+posx,&dx);	//得到卫星信噪比
            else break;
            slx++;
        }
        p=p1+1;//切换到下一个GPGSV信息
    }
}

/*----------------------------------------------------------
 + 实现功能：分析GNGGA信息
 + 调用参数功能：
 - gpsx:nmea信息结构体
 - buf:接收到的GPS数据缓冲区首地址
----------------------------------------------------------*/
void NMEA_GPGGA_Analysis(nmea_msg *gpsx,u8 *buf)
{
    u8 *p1,dx;
    u8 posx;
    p1=(u8*)strstr((const char *)buf,"$GNGGA");
    posx=NMEA_Comma_Pos(p1,6);								//得到GPS状态
    if(posx!=0XFF)gpsx->gpssta=NMEA_Str2num(p1+posx,&dx);
    posx=NMEA_Comma_Pos(p1,7);								//得到用于定位的卫星数
    if(posx!=0XFF)gpsx->posslnum=NMEA_Str2num(p1+posx,&dx);
    posx=NMEA_Comma_Pos(p1,9);								//得到海拔高度
    if(posx!=0XFF)gpsx->altitude=NMEA_Str2num(p1+posx,&dx);
}

/*----------------------------------------------------------
 + 实现功能：分析GNGSA信息
 + 调用参数功能：
 - gpsx:nmea信息结构体
 - buf:接收到的GPS数据缓冲区首地址
----------------------------------------------------------*/
void NMEA_GPGSA_Analysis(nmea_msg *gpsx,u8 *buf)
{
    u8 *p1,dx;
    u8 posx;
    u8 i;
    p1=(u8*)strstr((const char *)buf,"$GNGSA");
    posx=NMEA_Comma_Pos(p1,2);								//得到定位类型
    if(posx!=0XFF)gpsx->fixmode=NMEA_Str2num(p1+posx,&dx);
    for(i=0; i<12; i++)										//得到定位卫星编号
    {
        posx=NMEA_Comma_Pos(p1,3+i);
        if(posx!=0XFF)gpsx->possl[i]=NMEA_Str2num(p1+posx,&dx);
        else break;
    }
    posx=NMEA_Comma_Pos(p1,15);								//得到PDOP位置精度因子
    if(posx!=0XFF)gpsx->pdop=NMEA_Str2num(p1+posx,&dx);
    posx=NMEA_Comma_Pos(p1,16);								//得到HDOP位置精度因子
    if(posx!=0XFF)gpsx->hdop=NMEA_Str2num(p1+posx,&dx);
    posx=NMEA_Comma_Pos(p1,17);								//得到VDOP位置精度因子
    if(posx!=0XFF)gpsx->vdop=NMEA_Str2num(p1+posx,&dx);
}

/*----------------------------------------------------------
 + 实现功能：分析GNRMC信息
 + 调用参数功能：
 - gpsx:nmea信息结构体
 - buf:接收到的GPS数据缓冲区首地址
----------------------------------------------------------*/
void NMEA_GPRMC_Analysis(nmea_msg *gpsx,u8 *buf)
{
    u8 *p1,dx;
    u8 posx;
    u32 temp;
    double rs;
    p1=(u8*)strstr((const char *)buf,"GNRMC");//"$GNRMC",经常有&和GNRMC分开的情况,故只判断GPRMC.
    posx=NMEA_Comma_Pos(p1,1);								//得到UTC时间
    if(posx!=0XFF)
    {
        temp=NMEA_Str2num(p1+posx,&dx)/NMEA_Pow(10,dx);	 	//得到UTC时间,去掉ms
        gpsx->utc.hour=temp/10000;
        gpsx->utc.min=(temp/100)%100;
        gpsx->utc.sec=temp%100;
    }
    posx=NMEA_Comma_Pos(p1,3);								//得到纬度
    if(posx!=0XFF)
    {
        temp=NMEA_Str2num(p1+posx,&dx);
        gpsx->latitude=temp/NMEA_Pow(10,dx+2);	//得到°
        rs=temp%NMEA_Pow(10,dx+2);				//得到'
        gpsx->latitude=gpsx->latitude*NMEA_Pow(10,7)+(rs*NMEA_Pow(10,7-dx))/60;//转换为°
    }
    posx=NMEA_Comma_Pos(p1,4);								//南纬还是北纬
    if(posx!=0XFF)gpsx->nshemi=*(p1+posx);
    posx=NMEA_Comma_Pos(p1,5);								//得到经度
    if(posx!=0XFF)
    {
        temp=NMEA_Str2num(p1+posx,&dx);
        gpsx->longitude=temp/NMEA_Pow(10,dx+2);	//得到°
        rs=temp%NMEA_Pow(10,dx+2);				//得到'
        gpsx->longitude=gpsx->longitude*NMEA_Pow(10,7)+(rs*NMEA_Pow(10,7-dx))/60;//转换为°
    }
    posx=NMEA_Comma_Pos(p1,6);								//东经还是西经
    if(posx!=0XFF)gpsx->ewhemi=*(p1+posx);
    posx=NMEA_Comma_Pos(p1,9);								//得到UTC日期
    if(posx!=0XFF)
    {
        temp=NMEA_Str2num(p1+posx,&dx);		 				//得到UTC日期
        gpsx->utc.date=temp/10000;
        gpsx->utc.month=(temp/100)%100;
        gpsx->utc.year=2000+temp%100;
    }
}

/*----------------------------------------------------------
 + 实现功能：分析GNVTG信息
 + 调用参数功能：
 - gpsx:nmea信息结构体
 - buf:接收到的GPS数据缓冲区首地址
----------------------------------------------------------*/
void NMEA_GPVTG_Analysis(nmea_msg *gpsx,u8 *buf)
{
    u8 *p1,dx;
    u8 posx;
    p1=(u8*)strstr((const char *)buf,"$GNVTG");
    posx=NMEA_Comma_Pos(p1,7);								//得到地面速率
    if(posx!=0XFF)
    {
        gpsx->speed=NMEA_Str2num(p1+posx,&dx);
        if(dx<3)gpsx->speed*=NMEA_Pow(10,3-dx);	 	 		//确保扩大1000倍
    }
}

/*----------------------------------------------------------
 + 实现功能：提取NMEA-0183信息
 + 调用参数功能：
 - gpsx:nmea信息结构体
 - buf:接收到的GPS数据缓冲区首地址
----------------------------------------------------------*/
void GPS_Analysis(nmea_msg *gpsx,u8 *buf)
{
    NMEA_GPGSV_Analysis(gpsx,buf);	//GPGSV解析
    NMEA_GPGGA_Analysis(gpsx,buf);	//GNGGA解析
    NMEA_GPGSA_Analysis(gpsx,buf);	//GNGSA解析
    NMEA_GPRMC_Analysis(gpsx,buf);	//GNRMC解析
    NMEA_GPVTG_Analysis(gpsx,buf);	//GNVTG解析
}

/*----------------------------------------------------------
 + 实现功能：GPS校验和计算
 + 调用参数功能：
 - buf:数据缓存区首地址
 - len:数据长度
 - cka,ckb:两个校验结果.
----------------------------------------------------------*/
void Ublox_CheckSum(u8 *buf,u16 len,u8* cka,u8*ckb)
{
    u16 i;
    *cka=0;
    *ckb=0;
    for(i=0; i<len; i++)
    {
        *cka=*cka+buf[i];
        *ckb=*ckb+*cka;
    }
}

/*----------------------------------------------------------
 + 实现功能：检查CFG配置执行情况
 + 返回值:0,ACK成功;1,接收超时错误;2,没有找到同步字符;3,接收到NACK应答
----------------------------------------------------------*/
u8 Ublox_Cfg_Ack_Check(void)
{
    u16 len=0,i;
    u8 rval=0;
    while((GPSUSART_RX_STA&0X8000)==0 && len<100)//等待接收到应答
    {
        len++;
        Delay_ms(5);
    }
    if(len<250)   	//超时错误.
    {
        len=GPSUSART_RX_STA&0X7FFF;	//此次接收到的数据长度
        for(i=0; i<len; i++)if(GPSUSART_RX_BUF[i]==0XB5)break; //查找同步字符 0XB5
        if(i==len)rval=2;						//没有找到同步字符
        else if(GPSUSART_RX_BUF[i+3]==0X00)rval=3;//接收到NACK应答
        else rval=0;	   						//接收到ACK应答
    }
    else rval=1;								//接收超时错误
    GPSUSART_RX_STA=0;							//清除接收
    return rval;
}

/*----------------------------------------------------------
 + 实现功能：将当前配置保存在外部EEPROM里面
 + 返回值:0,执行成功;其他,执行失败
----------------------------------------------------------*/
u8 Ublox_Cfg_Cfg_Save(void)
{
    u8 i;
    _ublox_cfg_cfg *cfg_cfg=(_ublox_cfg_cfg *)GPSUSART_TX_BUF;
    cfg_cfg->header=0X62B5;		//cfg header
    cfg_cfg->id=0X0906;			//cfg cfg id
    cfg_cfg->dlength=13;		//数据区长度为13个字节.
    cfg_cfg->clearmask=0;		//清除掩码为0
    cfg_cfg->savemask=0XFFFF; 	//保存掩码为0XFFFF
    cfg_cfg->loadmask=0; 		//加载掩码为0
    cfg_cfg->devicemask=4; 		//保存在EEPROM里面
    Ublox_CheckSum((u8*)(&cfg_cfg->id),sizeof(_ublox_cfg_cfg)-4,&cfg_cfg->cka,&cfg_cfg->ckb);
    Ublox_Send_Date((u8*)cfg_cfg,sizeof(_ublox_cfg_cfg));//发送数据给GPS
    for(i=0; i<6; i++)if(Ublox_Cfg_Ack_Check()==0)break;		//EEPROM写入需要比较久时间,所以连续判断多次
    return i==6?1:0;
}

/*----------------------------------------------------------
 + 实现功能：配置NMEA输出信息格式
 + 调用参数功能：
 - msgid:要操作的NMEA消息条目,具体见下面的参数表
    00,GPGGA;01,GPGLL;02,GPGSA;
    03,GPGSV;04,GPRMC;05,GPVTG;
    06,GPGRS;07,GPGST;08,GPZDA;
    09,GPGBS;0A,GPDTM;0D,GPGNS;
 + 返回值:0,执行成功;其他,执行失败
----------------------------------------------------------*/
u8 Ublox_Cfg_Msg(u8 msgid,u8 uart1set)
{
    _ublox_cfg_msg *cfg_msg=(_ublox_cfg_msg *)GPSUSART_TX_BUF;
    cfg_msg->header=0X62B5;		//cfg header
    cfg_msg->id=0X0106;			//cfg msg id
    cfg_msg->dlength=8;			//数据区长度为8个字节.
    cfg_msg->msgclass=0XF0;  	//NMEA消息
    cfg_msg->msgid=msgid; 		//要操作的NMEA消息条目
    cfg_msg->iicset=1; 			//默认开启
    cfg_msg->uart1set=uart1set; //开关设置
    cfg_msg->uart2set=1; 	 	//默认开启
    cfg_msg->usbset=1; 			//默认开启
    cfg_msg->spiset=1; 			//默认开启
    cfg_msg->ncset=1; 			//默认开启
    Ublox_CheckSum((u8*)(&cfg_msg->id),sizeof(_ublox_cfg_msg)-4,&cfg_msg->cka,&cfg_msg->ckb);
    Ublox_Send_Date((u8*)cfg_msg,sizeof(_ublox_cfg_msg));//发送数据给GPS
    return Ublox_Cfg_Ack_Check();
}

/*----------------------------------------------------------
 + 实现功能：配置NMEA输出信息格式
 + 调用参数功能：
 - baudrate:波特率,4800/9600/19200/38400/57600/115200/230400
 + 返回值:0,执行成功;其他,执行失败
----------------------------------------------------------*/
u8 Ublox_Cfg_Prt(u32 baudrate)
{
    _ublox_cfg_prt *cfg_prt=(_ublox_cfg_prt *)GPSUSART_TX_BUF;
    cfg_prt->header=0X62B5;		//cfg header
    cfg_prt->id=0X0006;			//cfg prt id
    cfg_prt->dlength=20;		//数据区长度为20个字节.
    cfg_prt->portid=1;			//操作串口
    cfg_prt->reserved=0;	 	//保留字节,设置为0
    cfg_prt->txready=0;	 		//TX Ready设置为0
    cfg_prt->mode=0X08D0; 		//8位,1个停止位,无校验位
    cfg_prt->baudrate=baudrate; //波特率设置
    cfg_prt->inprotomask=0X0007;//0+1+2
    cfg_prt->outprotomask=0X0007;//0+1+2
    cfg_prt->reserved4=0; 		//保留字节,设置为0
    cfg_prt->reserved5=0; 		//保留字节,设置为0
    Ublox_CheckSum((u8*)(&cfg_prt->id),sizeof(_ublox_cfg_prt)-4,&cfg_prt->cka,&cfg_prt->ckb);
    Ublox_Send_Date((u8*)cfg_prt,sizeof(_ublox_cfg_prt));//发送数据给GPS
    Delay_ms(200);						//等待发送完成
    return Ublox_Cfg_Ack_Check();//这里不会反回0,因为UBLOX发回来的应答在串口重新初始化的时候已经被丢弃了.
}

/*----------------------------------------------------------
 + 实现功能：配置UBLOX GPS的时钟脉冲输出
 + 调用参数功能：
 - interval:脉冲间隔(us)
 - length:脉冲宽度(us)
 - status:脉冲配置:1,高电平有效;0,关闭;-1,低电平有效.
 + 返回值:0,发送成功;其他,发送失败
----------------------------------------------------------*/
u8 Ublox_Cfg_Tp(u32 interval,u32 length,signed char status)
{
    _ublox_cfg_tp *cfg_tp=(_ublox_cfg_tp *)GPSUSART_TX_BUF;
    cfg_tp->header=0X62B5;		//cfg header
    cfg_tp->id=0X0706;			//cfg tp id
    cfg_tp->dlength=20;			//数据区长度为20个字节.
    cfg_tp->interval=interval;	//脉冲间隔,us
    cfg_tp->length=length;		//脉冲宽度,us
    cfg_tp->status=status;	   	//时钟脉冲配置
    cfg_tp->timeref=0;			//参考UTC 时间
    cfg_tp->flags=0;			//flags为0
    cfg_tp->reserved=0;		 	//保留位为0
    cfg_tp->antdelay=820;    	//天线延时为820ns
    cfg_tp->rfdelay=0;    		//RF延时为0ns
    cfg_tp->userdelay=0;    	//用户延时为0ns
    Ublox_CheckSum((u8*)(&cfg_tp->id),sizeof(_ublox_cfg_tp)-4,&cfg_tp->cka,&cfg_tp->ckb);
    Ublox_Send_Date((u8*)cfg_tp,sizeof(_ublox_cfg_tp));//发送数据给GPS
    return Ublox_Cfg_Ack_Check();
}

/*----------------------------------------------------------
 + 实现功能：配置UBLOX GPS的更新速率
 + 调用参数功能：
 - measrate:测量时间间隔，单位为ms，最少不能小于200ms（5Hz）
 - reftime:参考时间，0=UTC Time；1=GPS Time（一般设置为1）
 + 返回值:0,发送成功;其他,发送失败
----------------------------------------------------------*/
u8 Ublox_Cfg_Rate(u16 measrate,u8 reftime)
{
    _ublox_cfg_rate *cfg_rate=(_ublox_cfg_rate *)GPSUSART_TX_BUF;
    if(measrate<100)return 1;	//小于200ms，直接退出
    cfg_rate->header=0X62B5;	//cfg header
    cfg_rate->id=0X0806;	 	//cfg rate id
    cfg_rate->dlength=6;	 	//数据区长度为6个字节.
    cfg_rate->measrate=measrate;//脉冲间隔,us
    cfg_rate->navrate=1;		//导航速率（周期），固定为1
    cfg_rate->timeref=reftime; 	//参考时间为GPS时间
    Ublox_CheckSum((u8*)(&cfg_rate->id),sizeof(_ublox_cfg_rate)-4,&cfg_rate->cka,&cfg_rate->ckb);
    Ublox_Send_Date((u8*)cfg_rate,sizeof(_ublox_cfg_rate));//发送数据给GPS
    return Ublox_Cfg_Ack_Check();
}

/*----------------------------------------------------------
 + 实现功能：GPS模块初始化
----------------------------------------------------------*/
void GPS_Init()
{
    /* 串口4初始化，函数参数为波特率 */
    Device_Usart4_ENABLE_Init(9600,3,0,DISABLE,ENABLE);
}

/*----------------------------------------------------------
 + 实现功能：由任务调度调用周期100ms
----------------------------------------------------------*/
/* NMEA 0183 协议解析后数据存放结构体 */
nmea_msg gpsx;
void Call_GPS()
{
    /* 数据长度计算的数组下标 */
    u16 i,rxlen;
	
	if(++GPS_DOG > 10)//如果1s钟还没有接收到GPS的数据则认为GPS丢失
	{
		GPS_DOG = 11;//数据限幅防止数据溢出
		memset( &gpsx, 0, sizeof(gpsx) );//将GPS结构体内的数据全部清零
	}
	/* 接收到400字节以上数据后解析 */
    else if(GPSUSART_RX_STA>400)
    {
        /* 得到数据长度 */
        rxlen=GPSUSART_RX_STA;
        /* 记录为上一次的静态数据 */
        for(i=0; i<rxlen; i++)GPSUSART_RX_BUF_old[i]=GPSUSART_RX_BUF[i];
        /* 添加结束符 */
        GPSUSART_RX_BUF_old[i]=0;
        /* 继续下一次接收 */
        GPSUSART_RX_STA=0;
        /* 分析静态数据字符串 */
        GPS_Analysis(&gpsx,(u8*)GPSUSART_RX_BUF_old);//
    }
}

/*----------------------------------------------------------
 + 实现功能：由串口接收中断调用
----------------------------------------------------------*/
void GPS_Get(u8 data)
{
    /* 还可以接收数据 */
    if(GPSUSART_RX_STA<GPSUSART_MAX_RECV_LEN)
    {
        /* 记录接收到的值 */
        GPSUSART_RX_BUF[GPSUSART_RX_STA++]=data;
    }
    /* 接收数据已满 */
    else
    {
        /* 重置数组下标 */
        GPSUSART_RX_STA=0;
        /* 记录接收到的值 */
        GPSUSART_RX_BUF[GPSUSART_RX_STA++]=data;
    }
}

