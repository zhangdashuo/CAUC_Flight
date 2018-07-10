/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file Optical_Flow.c
*@version V1.0
*@date  2018/5/24
*@brief 光流数据处理文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#include "stm32f4xx.h"
#include "Optical_Flow.h"
#include "driver_usart.h"
#include "driver_ultrasonic.h"
#include "driver_mpu6050.h"
#include "math.h"


/*当图片大小为 B64*64时的光流定点*/
/*每一度对应多少个像素*/
#define  Roll_Compensation     64.0f/(2.0f*atan(32.0f/60.0f)*57.3f)
#define  Pitch_Compensation    64.0f/(2.0f*atan(32.0f/60.0f)*57.3f)

/* 发送帧头 接收帧头 */
#define title1_received 0xAA
#define title2_received 0xAE

/* 滑动窗口滤波数值个数 */
#define OF_FILTER_NUM 					10

/* 滤波滑动窗口数组 */
float OF_FILT_BUF[2][(OF_FILTER_NUM + 1)];

/* 滤波滑动窗口数组下标 */
uint8_t of_filter_cnt = 0,of_filter_cnt_old = 0;

/* 读取X偏移量 读取Y偏移量 读取置信度 */
int Optical_flow_org_x,Optical_flow_org_y,Optical_flow_confidence;
float Optical_flow_x,Optical_flow_y,Optical_flow_con;

/* 光流通过陀螺移和超声波补偿后的速度 */
float Flow_Comp_x,Flow_Comp_y;
/* 光流通过超声波补偿后的速度 */
float no_compen_x_speed,no_compen_y_speed;
/* 光流积分位移 */
float OF_Offset_x ,OF_Offset_y;

/*----------------------------------------------------------
 + 实现功能：数据分析
 + 调用参数：接收到的单字节数据
----------------------------------------------------------*/
void Optical_Flow_Receive_Anl(u8 *data_buf,u8 num)
{
	/* 计算的速度临时存放变量 */
	float temp_comp_x,temp_comp_y;
	/* 滑动滤波数组 */
	float OF_FILT_TMP[2] = {0,0};
	
	
	/* 读取X偏移量原始数据  */
	Optical_flow_org_x = (int)(*(data_buf+1)<<0) | (int)(*(data_buf+2)<<8) | (int)(*(data_buf+3)<<16) | (int)(*(data_buf+4)<<24) ;
	/* 换算到X像素速度  光流像素要结合高度才能算出速度 值范围约+-200.000 */
	Optical_flow_y = (float)((double)Optical_flow_org_x*0.0001);

	/* 读取Y偏移量原始数据 */
	Optical_flow_org_y = (int)(*(data_buf+5)<<0) | (int)(*(data_buf+6)<<8) | (int)(*(data_buf+7)<<16) | (int)(*(data_buf+8)<<24) ;
	/* 换算到Y像素速度  光流像素要结合高度才能算出速度 值范围约+-200.000 */
	Optical_flow_x = -(float)((double)Optical_flow_org_y*0.0001);

	/* 读取Confidence原始数据 */
	Optical_flow_confidence = (int)(*(data_buf+9)<<0) | (int)(*(data_buf+10)<<8) | (int)(*(data_buf+11)<<16) | (int)(*(data_buf+12)<<24) ;
    /* 读取画面不变率 值范围0-100 */
    Optical_flow_con = (float)((double)Optical_flow_confidence*0.001);
	
	//像素点的位移之和转换成实际的距离,实际测试，B64x64的图片   高度50cm 60cm对应60个像素点
	no_compen_x_speed=(float)Optical_flow_x*(float)ultra_distance/600.0f/0.05f;
	no_compen_y_speed=(float)Optical_flow_y*(float)ultra_distance/600.0f/0.05f;
	//计算对应的速度. my_data[1]=移动像素点*10的倍数  T=0.05s    陀螺仪角速度单位 degree/s 
	temp_comp_x=((float)Optical_flow_x/10.0f-Roll_Compensation*(-mpu6050.Gyro_deg.x)*0.05f)*(float)ultra_distance/600.0f*10.0f/0.05f;//mm/s 
	temp_comp_y=((float)Optical_flow_y/10.0f-Pitch_Compensation*(-mpu6050.Gyro_deg.y)*0.05f)*(float)ultra_distance/600.0f*10.0f/0.05f;
	
	/* 滑动窗口滤波数组下标移位 */
	if( ++of_filter_cnt > OF_FILTER_NUM )
	{
		of_filter_cnt = 0;
		of_filter_cnt_old = 1;
	}
	else
		of_filter_cnt_old = (of_filter_cnt == OF_FILTER_NUM)? 0 : (of_filter_cnt + 1);
	
	/* 更新滤波滑动窗口临时数组 */
	OF_FILT_BUF[0][of_filter_cnt] = temp_comp_x;
	OF_FILT_BUF[1][of_filter_cnt] = temp_comp_y;
	
	/* 更新滤波滑动窗口数组 */
	for(u8 i=0; i<OF_FILTER_NUM; i++)
	{
		OF_FILT_TMP[0] += OF_FILT_BUF[0][i];
		OF_FILT_TMP[1] += OF_FILT_BUF[1][i];
	}
	
	/* 得出处理后的数据 */
	Flow_Comp_x  = ((float)( OF_FILT_TMP[0] )/(float)OF_FILTER_NUM);	
	Flow_Comp_y  = ((float)( OF_FILT_TMP[1] )/(float)OF_FILTER_NUM);	
	
	OF_Offset_x += Flow_Comp_x/1000;
	OF_Offset_y += Flow_Comp_y/1000;
}

/*----------------------------------------------------------
 + 实现功能：数据接收并保存
 + 调用参数：接收到的单字节数据
----------------------------------------------------------*/
void Optical_Flow_Receive_Prepare(u8 data)
{
    /* 局部静态变量：接收缓存 */
    static u8 RxBuffer[14];
    /* 数据长度 *//* 数据数组下标 */
    static u8  _data_cnt = 0;
    /* 接收状态 */
    static u8 state = 0;

    /* 帧头1 */
    if(state==0&&data==title1_received)
    {
        state=1;
    }
    /* 帧头2 */
    else if(state==1&&data==title2_received)
    {
        state=2;
		_data_cnt = 0;
    }
    /* 接收数据租 */
    else if(state==2)
    {
        RxBuffer[++_data_cnt]=data;
        if(_data_cnt>=12)
		{
			state = 0;
			Optical_Flow_Receive_Anl(RxBuffer,_data_cnt);
		}
    }
    /* 若有错误重新等待接收帧头 */
    else
        state = 0;
}


/*----------------------------------------------------------
 + 实现功能：光流初始化
----------------------------------------------------------*/
void Optical_Flow_init()
{
    /*----------------------------------------------------------
     + 实现功能：串口3初始化
     + 调用参数功能：
     - u32 bound：波特率 115200
     - u8 Priority：中断主优先级 2
     - u8 SubPriority：中断从优先级 0
     - FunctionalState TXenable：发送中断使能 失能
     - FunctionalState RXenable：就收中断使能 使能
    ----------------------------------------------------------*/
    Device_Usart3_ENABLE_Init(115200,2,0,DISABLE,ENABLE);
}

