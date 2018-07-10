/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.
 + 文件名  ：Optical_Flow.c
 + 描述    ：光流数据处理
代码尽量做到逐行注释 代码有问题，及时加群交流 作者有偿支持对开源代码的完善 */
#include "stm32f4xx.h"
#include "Optical_Flow.h"
#include "driver_usart.h"

/* 发送帧头 接收帧头 */
#define title1_received 0xAA
#define title2_received 0xAE

/* 读取X偏移量 读取Y偏移量 读取置信度 */
int Optical_flow_org_x,Optical_flow_org_y,Optical_flow_confidence;
float Optical_flow_x,Optical_flow_y,Optical_flow_con;


/*加入PIX光流驱动*/
FLOW flow;
FLOW_RAD flow_rad;
FLOW_FIX flow_fix;
u8 FLOW_STATE[4];
u8 flow_buf[27];
u8 flow_buf_rad[45];

float ByteToFloat(u8* byteArry)
{
  return *((float*)byteArry);
}

/*====================================================================================================*/
/*====================================================================================================*
** 函数名称: IIR_I_Filter
** 功能描述: IIR直接I型滤波器
** 输    入: InData 为当前数据
**           *x     储存未滤波的数据
**           *y     储存滤波后的数据
**           *b     储存系数b
**           *a     储存系数a
**           nb     数组*b的长度
**           na     数组*a的长度
**           LpfFactor
** 输    出: OutData         
** 说    明: 无
** 函数原型: y(n) = b0*x(n) + b1*x(n-1) + b2*x(n-2) -
                    a1*y(n-1) - a2*y(n-2)
**====================================================================================================*/
/*====================================================================================================*/
double IIR_I_Filter(double InData, double *x, double *y, double *b, short nb, double *a, short na)
{
  double z1,z2;
  short i;
  double OutData;
  
  for(i=nb-1; i>1; i--)
  {
    x[i]=x[i-1];
  }
  
  x[0] = InData;
  
  for(z1=0,i=0; i<nb; i++)
  {
    z1 += x[i]*b[i];
  }
  
  for(i=na-1; i>1; i--)
  {
    y[i]=y[i-1];
  }
  
  for(z2=0,i=1; i<na; i++)
  {
    z2 += y[i]*a[i];
  }
  
  y[0] = z1 - z2; 
  OutData = y[0];
    
  return OutData;
}

#define  IIR_ORDER     4      //使用IIR滤波器的阶数
static double b_IIR[IIR_ORDER+1] ={0.0008f, 0.0032f, 0.0048f, 0.0032f, 0.0008f};  //系数b
static double a_IIR[IIR_ORDER+1] ={1.0000f, -3.0176f, 3.5072f, -1.8476f, 0.3708f};//系数a
static double InPut_IIR[4][IIR_ORDER+1] = {0};
static double OutPut_IIR[4][IIR_ORDER+1] = {0};
      
u8 get_one_fame=0;

/*----------------------------------------------------------
 + 实现功能：数据分析
 + 调用参数：接收到的单字节数据
----------------------------------------------------------*/
//void Optical_Flow_Receive_Anl(u8 *data_buf,u8 num)
void Optical_Flow_Receive_Anl(void)
{
	u8 floattobyte[4];          

	if(get_one_fame)
	{
		if(FLOW_STATE[3]==100)
		{
			flow.time_sec=((uint64_t)flow_buf[6]<<56)|((uint64_t)flow_buf[5]<<48)|((uint64_t)flow_buf[4]<<40)\
			|((uint64_t)flow_buf[3]<<32)|(flow_buf[2]<<16)|(flow_buf[1]<<8)|(flow_buf[0]);
			floattobyte[0]=flow_buf[8];
			floattobyte[1]=flow_buf[9];
			floattobyte[2]=flow_buf[10];
			floattobyte[3]=flow_buf[11];
			flow.flow_comp_x.originf =ByteToFloat(floattobyte);
			floattobyte[0]=flow_buf[12];
			floattobyte[1]=flow_buf[13];
			floattobyte[2]=flow_buf[14];
			floattobyte[3]=flow_buf[15];
			flow.flow_comp_y.originf =ByteToFloat(floattobyte);
			floattobyte[0]=flow_buf[16];
			floattobyte[1]=flow_buf[17];
			floattobyte[2]=flow_buf[18];
			floattobyte[3]=flow_buf[19];
			flow.hight.originf=ByteToFloat(floattobyte);//ground_distance        float        Ground distance in m. Positive value: distance known. Negative value: Unknown distance     
			flow.flow_x.origin=(int16_t)((flow_buf[20])|(flow_buf[21]<<8));
			flow.flow_y.origin=(int16_t)((flow_buf[22])|(flow_buf[23]<<8));
			flow.id=flow_buf[24];
			flow.quality=flow_buf[25]; //Optical flow quality / confidence. 0: bad, 255: maximum quality
			flow_fix.flow_x.average = IIR_I_Filter(flow_fix.flow_x.origin , InPut_IIR[0], OutPut_IIR[0], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
			flow.flow_y.average = IIR_I_Filter(flow.flow_y.origin , InPut_IIR[1], OutPut_IIR[1], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
			flow.flow_comp_x.average = IIR_I_Filter(flow.flow_comp_x.originf , InPut_IIR[2], OutPut_IIR[2], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
			flow.flow_comp_y.average = IIR_I_Filter(flow.flow_comp_y.originf , InPut_IIR[3], OutPut_IIR[3], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
		}
		else if(FLOW_STATE[3]==106)
		{
			flow_rad.time_usec=((uint64_t)flow_buf_rad[6]<<56)|((uint64_t)flow_buf_rad[5]<<48)|((uint64_t)flow_buf_rad[4]<<40)|\
			((uint64_t)flow_buf_rad[3]<<32)|(flow_buf_rad[2]<<16)|(flow_buf_rad[1]<<8)|(flow_buf_rad[0]);
			flow_rad.integration_time_us=((uint64_t)flow_buf_rad[11]<<32)|(flow_buf_rad[10]<<16)|(flow_buf_rad[9]<<8)|(flow_buf_rad[8]);
			floattobyte[0]=flow_buf_rad[12];
			floattobyte[1]=flow_buf_rad[13];
			floattobyte[2]=flow_buf_rad[14];
			floattobyte[3]=flow_buf_rad[15];
			flow_rad.integrated_x=ByteToFloat(floattobyte);
			floattobyte[0]=flow_buf_rad[16];
			floattobyte[1]=flow_buf_rad[17];
			floattobyte[2]=flow_buf_rad[18];
			floattobyte[3]=flow_buf_rad[19];
			flow_rad.integrated_y=ByteToFloat(floattobyte);
			floattobyte[0]=flow_buf_rad[20];
			floattobyte[1]=flow_buf_rad[21];
			floattobyte[2]=flow_buf_rad[22];
			floattobyte[3]=flow_buf_rad[23];
			flow_rad.integrated_xgyro=ByteToFloat(floattobyte);
			floattobyte[0]=flow_buf_rad[24];
			floattobyte[1]=flow_buf_rad[25];
			floattobyte[2]=flow_buf_rad[26];
			floattobyte[3]=flow_buf_rad[27];
			flow_rad.integrated_ygyro=ByteToFloat(floattobyte);
			floattobyte[0]=flow_buf_rad[28];
			floattobyte[1]=flow_buf_rad[29];
			floattobyte[2]=flow_buf_rad[30];
			floattobyte[3]=flow_buf_rad[31];
			flow_rad.integrated_zgyro=ByteToFloat(floattobyte);
			flow_rad.time_delta_distance_us=(flow_buf_rad[35]<<24)|(flow_buf_rad[34]<<16)|(flow_buf_rad[33]<<8)|(flow_buf_rad[32]);
			floattobyte[0]=flow_buf_rad[36];
			floattobyte[1]=flow_buf_rad[37];
			floattobyte[2]=flow_buf_rad[38];
			floattobyte[3]=flow_buf_rad[39];
			flow_rad.distance=ByteToFloat(floattobyte);
			flow_rad.temperature=(flow_buf_rad[41]<<8)|(flow_buf_rad[40]);
			flow_rad.sensor_id=(flow_buf_rad[42]);
			flow_rad.quality=(flow_buf_rad[43]);
                 
			flow_fix.flow_x.origin =flow.flow_x.origin + flow_rad.integrated_ygyro*flow_fix.scale_rad_fix;
			flow_fix.flow_y.origin =flow.flow_y.origin - flow_rad.integrated_xgyro*flow_fix.scale_rad_fix;
			flow_fix.flow_comp_x.originf =flow.flow_comp_x.originf - flow_rad.integrated_ygyro*flow_fix.scale_rad_fix_comp;
			flow_fix.flow_comp_y.originf =flow.flow_comp_y.originf + flow_rad.integrated_xgyro*flow_fix.scale_rad_fix_comp;
		}       
	}
}

/*----------------------------------------------------------
 + 实现功能：数据接收并保存
 + 调用参数：接收到的单字节数据
----------------------------------------------------------*/
void Optical_Flow_Receive_Prepare(u8 data)
{
	
	static u8 s_flow=0,data_cnt=0;
	
	switch(s_flow)
	{
		case 0: 
			if(data==0xFE)
				s_flow=1;
		break;
		case 1: 
			if(data==0x1A||data==0x2C)
				s_flow=2;
			else
				s_flow=0;
		break;
		case 2:
			if(data_cnt<4)
				{s_flow=2; FLOW_STATE[data_cnt++]=data;}
			else
				{data_cnt=0;s_flow=3;flow_buf[data_cnt++]=data;}
		break;
		case 3:
			if(FLOW_STATE[3]==100)
				{
					if(data_cnt<26)
						{s_flow=3; flow_buf[data_cnt++]=data;}
					else
						{data_cnt=0;s_flow=4;}
                }
            else if(FLOW_STATE[3]==106)
				{
					if(data_cnt<44)
						{s_flow=3; flow_buf_rad[data_cnt++]=data;}
					else
						{data_cnt=0;s_flow=4;}
				}
            else
				{data_cnt=0;s_flow=0;}
		break;
		case 4:
			get_one_fame=1;s_flow=0;data_cnt=0;
		break;
		default:
			s_flow=0;data_cnt=0;
		break;
	}//--end of s_uart
	Optical_Flow_Receive_Anl();
}


/*----------------------------------------------------------
 + 实现功能：光流初始化
----------------------------------------------------------*/
void Optical_Flow_init()
{
    /*----------------------------------------------------------
     + 实现功能：串口3初始化
     + 调用参数功能：
     - u32 bound：波特率 38400
     - u8 Priority：中断主优先级 2
     - u8 SubPriority：中断从优先级 0
     - FunctionalState TXenable：发送中断使能 失能
     - FunctionalState RXenable：就收中断使能 使能
    ----------------------------------------------------------*/
    Device_Usart3_ENABLE_Init(115200,2,0,DISABLE,ENABLE);
}

/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
