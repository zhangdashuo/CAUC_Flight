/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file data_transfer.c
*@version V1.0
*@date  2018/5/24
*@brief 数据传输文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#include "stm32f4xx.h"
#include "data_transfer.h"
#include "driver_usart.h"
#include "imu.h"
#include "driver_mpu6050.h"
#include "driver_ak8975.h"
#include "driver_ms5611.h"
#include "rc.h"
#include "ctrl.h"
#include "time.h"
#include "driver_ultrasonic.h"
#include "Optical_Flow.h"
#include "driver_GPS.h"
#include "position_control.h"
#include "usbd_user_hid.h"
#include "driver_power.h"
#include "OFposition_control.h"

/* 数据拆分宏定义，在发送大于8位的数据类型时，比如int16、int32等，需要把数据拆分成8位逐个发送 */
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp) + 0) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
/* 发送帧头 接收帧头 */
#define title1_send 0xAA
#define title2_send 0xAA
#define title1_received 0xAA
#define title2_received 0xAF

/* 等待发送数据的标志 */
u8 wait_for_translate;
/* 等待发送数据的标志 */
dt_flag_t f;
/* 发送数据缓存数组 */
u8 data_to_send[50];
/* 是否写入并保存数据 */
u16 flash_save_en_cnt = 0;
/* 8通道输入数传控制数据的数组1000-2000的控制信号 */
u16 RX_CH[8];

/*----------------------------------------------------------
 + 实现功能：数传数据发送
 + 调用参数：要发送的数据组 数据长度
----------------------------------------------------------*/
void DT_Send_Data(u8 *dataToSend , u8 length)
{
    /* 串口2发送 要发送的数据组 数据长度 */
    if(wait_for_translate)
	{
#ifdef DT_USE_USB_HID
		Usb_Hid_Adddata(data_to_send,length);
#endif
#ifdef DT_USE_USART2
		Usart2_Send(data_to_send, length);
#endif
	}
}

/*----------------------------------------------------------
 + 实现功能：校验累加和回传
 + 调用参数：字帧 校验累加和
----------------------------------------------------------*/
static void DT_Send_Check(u8 head, u8 check_sum)
{
    /* 数据内容 */
    data_to_send[0]=title1_send;
    data_to_send[1]=title2_send;
    data_to_send[2]=0xEF;
    data_to_send[3]=2;
    data_to_send[4]=head;
    data_to_send[5]=check_sum;

    /* 校验累加和计算 */
    u8 sum = 0;
    for(u8 i=0; i<6; i++)
        sum += data_to_send[i];
    data_to_send[6]=sum;
    /* 发送 要发送的数据组 数据长度 */
    DT_Send_Data(data_to_send, 7);
}

/*----------------------------------------------------------
 + 实现功能：数据分析
 + 调用参数：接收到的单字节数据
----------------------------------------------------------*/
void DT_Data_Receive_Anl(u8 *data_buf,u8 num)
{
    u8 sum = 0;
    /* 计算校验累加和 */
    for(u8 i=0; i<(num-1); i++)
        sum += *(data_buf+i);
    /* 判断校验累加和 */
    if(!(sum==*(data_buf+num-1)))		return;
    /* 判断帧头 */
    if(!(*(data_buf)==title1_received && *(data_buf+1)==title2_received))		return;
    /* 判断功能字：主要命令集 */
    if(*(data_buf+2)==0X01)
    {
        /* 加速度计校准 */
        if(*(data_buf+4)==0X01)
        {
            mpu6050.Acc_CALIBRATE = 1;
            start_height=0;
        }
        /* 陀螺仪校准 */
        else if(*(data_buf+4)==0X02)
        {
            mpu6050.Gyro_CALIBRATE = 1;
            start_height=0;
        }
        /* 加速度计和陀螺仪校准 */
        else if(*(data_buf+4)==0X03)
        {
            mpu6050.Acc_CALIBRATE = 1;
            mpu6050.Gyro_CALIBRATE = 1;
            start_height=0;
        }
        /* 磁力计校准 */
        else if(*(data_buf+4)==0X04)
            Mag_CALIBRATED = 1;
        /* 飞控锁定 */
        else if(*(data_buf+4)==0XA0)
            unlocked_to_fly=0;
        /* 飞控解锁 */
        else if(*(data_buf+4)==0XA1)
            unlocked_to_fly=1;
        /* PWM正常输出 */
        else if(*(data_buf+4)==0XD0)
            PWM_Mode=0;
        /* PWM全小输出 */
        else if(*(data_buf+4)==0XD1)
            PWM_Mode=1;
        /* PWM全大输出 */
        else if(*(data_buf+4)==0XD2)
            PWM_Mode=2;
    }
    /* 判断功能字：次要命令集 */
    if(*(data_buf+2)==0X02)
    {
        /* 读取全部10组PID参数 */
        if(*(data_buf+4)==0X01)
        {
            f.send_pid1 = 1;
            f.send_pid2 = 1;
            f.send_pid3 = 1;
            f.send_pid4 = 1;
        }
        /* 恢复默认PID参数 */
        if(*(data_buf+4)==0XA1)
        {
            Para_ResetToFactorySetup();
        }
    }
    /* 判断功能字 接收数据 */
    if(*(data_buf+2)==0X03)
    {
        /* 调用数传控制看门狗 */
        Call_RadioControl_Sign(2);
        /* 数据赋值 */
        RX_CH[THR] = (vs16)(*(data_buf+4)<<8)|*(data_buf+5) ;
        RX_CH[YAW] = (vs16)(*(data_buf+6)<<8)|*(data_buf+7) ;
        RX_CH[ROL] = (vs16)(*(data_buf+8)<<8)|*(data_buf+9) ;
        RX_CH[PIT] = (vs16)(*(data_buf+10)<<8)|*(data_buf+11) ;
        RX_CH[AUX1] = (vs16)(*(data_buf+12)<<8)|*(data_buf+13) ;
        RX_CH[AUX2] = (vs16)(*(data_buf+14)<<8)|*(data_buf+15) ;
        RX_CH[AUX3] = (vs16)(*(data_buf+16)<<8)|*(data_buf+17) ;
        RX_CH[AUX4] = (vs16)(*(data_buf+18)<<8)|*(data_buf+19) ;
    }
    /* PID数组：1,2,3 */
    if(*(data_buf+2)==0X10)
    {
        /* 数据赋值 */
        ctrl_angular_velocity.PID[PIDROLL].kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        ctrl_angular_velocity.PID[PIDROLL].ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        ctrl_angular_velocity.PID[PIDROLL].kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        ctrl_angular_velocity.PID[PIDPITCH].kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        ctrl_angular_velocity.PID[PIDPITCH].ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        ctrl_angular_velocity.PID[PIDPITCH].kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        ctrl_angular_velocity.PID[PIDYAW].kp 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        ctrl_angular_velocity.PID[PIDYAW].ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        ctrl_angular_velocity.PID[PIDYAW].kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        DT_Send_Check(*(data_buf+2),sum);
        /* 立即重重当前接收到数据 */
        PID_Para_Init();
        /* 写入并保存数据 */
        flash_save_en_cnt = 1;
    }
    /* PID数组：4,5,6 */
    if(*(data_buf+2)==0X11)
    {
        /* 数据赋值 */
        ctrl_attitude.PID[PIDROLL].kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        ctrl_attitude.PID[PIDROLL].ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        ctrl_attitude.PID[PIDROLL].kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        ctrl_attitude.PID[PIDPITCH].kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        ctrl_attitude.PID[PIDPITCH].ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        ctrl_attitude.PID[PIDPITCH].kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        ctrl_attitude.PID[PIDYAW].kp 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        ctrl_attitude.PID[PIDYAW].ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        ctrl_attitude.PID[PIDYAW].kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        DT_Send_Check(*(data_buf+2),sum);
        /* 立即重重当前接收到数据 */
        PID_Para_Init();
        /* 写入并保存数据 */
        flash_save_en_cnt = 1;
    }
    /* PID数组：7,8,9 */
    if(*(data_buf+2)==0X12)
    {
        /* 数据赋值 */
        pid_setup.groups.hc_sp.kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        pid_setup.groups.hc_sp.ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        pid_setup.groups.hc_sp.kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        pid_setup.groups.hc_height.kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        pid_setup.groups.hc_height.ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        pid_setup.groups.hc_height.kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        pid_setup.groups.ctrl3.kp 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        pid_setup.groups.ctrl3.ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        pid_setup.groups.ctrl3.kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        DT_Send_Check(*(data_buf+2),sum);
        /* 立即重重当前接收到数据 */
        PID_Para_Init();
        /* 写入并保存数据 */
        flash_save_en_cnt = 1;
    }
    /* PID数组：10 */
    if(*(data_buf+2)==0X13)
    {
        /* 数据赋值 */
        pid_setup.groups.ctrl4.kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        pid_setup.groups.ctrl4.ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        pid_setup.groups.ctrl4.kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
		pid_setup.groups.ctrl5.kp  = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        pid_setup.groups.ctrl5.ki  = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        pid_setup.groups.ctrl5.kd  = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        pid_setup.groups.ctrl6.kp  = 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        pid_setup.groups.ctrl6.ki  = 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        pid_setup.groups.ctrl6.kd  = 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );

        DT_Send_Check(*(data_buf+2),sum);
        /* 立即重重当前接收到数据 */
        PID_Para_Init();
        /* 写入并保存数据 */
        flash_save_en_cnt = 1;
    }
    /* 回传校验累加和 */
    if(*(data_buf+2)==0X14)
    {
        DT_Send_Check(*(data_buf+2),sum);
    }
    /* 回传校验累加和 */
    if(*(data_buf+2)==0X15)
    {
        DT_Send_Check(*(data_buf+2),sum);
    }
}

/*----------------------------------------------------------
 + 实现功能：数据接收并保存
 + 调用参数：接收到的单字节数据
----------------------------------------------------------*/
void DT_Data_Receive_Prepare(u8 data)
{
    /* 局部静态变量：接收缓存 */
    static u8 RxBuffer[50];
    /* 数据长度 *//* 数据数组下标 */
    static u8 _data_len = 0,_data_cnt = 0;
    /* 接收状态 */
    static u8 state = 0;

    /* 帧头1 */
    if(state==0&&data==title1_received)
    {
        state=1;
        RxBuffer[0]=data;
    }
    /* 帧头2 */
    else if(state==1&&data==title2_received)
    {
        state=2;
        RxBuffer[1]=data;
    }
    /* 功能字 */
    else if(state==2&&data<0XF1)
    {
        state=3;
        RxBuffer[2]=data;
    }
    /* 长度 */
    else if(state==3&&data<50)
    {
        state = 4;
        RxBuffer[3]=data;
        _data_len = data;
        _data_cnt = 0;
    }
    /* 接收数据租 */
    else if(state==4&&_data_len>0)
    {
        _data_len--;
        RxBuffer[4+_data_cnt++]=data;
        if(_data_len==0)
            state = 5;
    }
    /* 校验累加和 */
    else if(state==5)
    {
        state = 0;
        RxBuffer[4+_data_cnt]=data;
        DT_Data_Receive_Anl(RxBuffer,_data_cnt+5);
    }
    /* 若有错误重新等待接收帧头 */
    else
        state = 0;
}

/*----------------------------------------------------------
 + 实现功能：发送速度信息
 + 调用参数：向北速度 向西速度 向上速度
 + 数量级：这里函数调用发送多少上位机就显示多少，保留2位小数
----------------------------------------------------------*/
void DT_Send_Speed(float x_s,float y_s,float z_s)
{
    u8 _cnt=0;
    vs16 _temp;

    data_to_send[_cnt++]=title1_send;
    data_to_send[_cnt++]=title2_send;
    data_to_send[_cnt++]=0x0B;
    data_to_send[_cnt++]=0;

    _temp = (int)(x_s*100.0f);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int)(y_s*100.0f);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int)(z_s*100.0f);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);


    data_to_send[3] = _cnt-4;

    u8 sum = 0;
    for(u8 i=0; i<_cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++]=sum;

    DT_Send_Data(data_to_send, _cnt);
}

/*----------------------------------------------------------
 + 实现功能：发送位置信息
 + 调用参数：可见卫星数、经度坐标、纬度坐标、悬停飞行模式 其中坐标信息 放大倍率10E5
----------------------------------------------------------*/
void DT_Send_Location(u8 state,u8 sat_num,s32 lon,s32 lat,s16 mode)
{
    u8 _cnt=0;
    vs16 _temp;
    vs32 _temp2;

    data_to_send[_cnt++]=title1_send;
    data_to_send[_cnt++]=title2_send;
    data_to_send[_cnt++]=0x04;
    data_to_send[_cnt++]=0;

    data_to_send[_cnt++]=state;//定位状态
    data_to_send[_cnt++]=sat_num;//可见卫星数

    _temp2 = lon;//经度
    data_to_send[_cnt++]=BYTE3(_temp2);
    data_to_send[_cnt++]=BYTE2(_temp2);
    data_to_send[_cnt++]=BYTE1(_temp2);
    data_to_send[_cnt++]=BYTE0(_temp2);

    _temp2 = lat;//纬度
    data_to_send[_cnt++]=BYTE3(_temp2);
    data_to_send[_cnt++]=BYTE2(_temp2);
    data_to_send[_cnt++]=BYTE1(_temp2);
    data_to_send[_cnt++]=BYTE0(_temp2);

    _temp = mode;//悬停飞行模式
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    data_to_send[3] = _cnt-4;

    u8 sum = 0;
    for(u8 i=0; i<_cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++]=sum;

    DT_Send_Data(data_to_send, _cnt);
}

/*----------------------------------------------------------
 + 实现功能：发送姿态信息
 + 调用参数：横滚、俯仰、航向、气压高度 厘米、控制高度模式、解锁状态
----------------------------------------------------------*/
void DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)
{
    u8 _cnt=0;
    vs16 _temp;
    vs32 _temp2 = alt;

    data_to_send[_cnt++]=title1_send;
    data_to_send[_cnt++]=title2_send;
    data_to_send[_cnt++]=0x01;
    data_to_send[_cnt++]=0;

    _temp = (int)(angle_rol*100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int)(angle_pit*100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int)(angle_yaw*100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    data_to_send[_cnt++]=BYTE3(_temp2);
    data_to_send[_cnt++]=BYTE2(_temp2);
    data_to_send[_cnt++]=BYTE1(_temp2);
    data_to_send[_cnt++]=BYTE0(_temp2);

    data_to_send[_cnt++] = fly_model;

    data_to_send[_cnt++] = armed;

    data_to_send[3] = _cnt-4;

    u8 sum = 0;
    for(u8 i=0; i<_cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++]=sum;

    DT_Send_Data(data_to_send, _cnt);
}

/*----------------------------------------------------------
 + 实现功能：发送姿态传感器信息
 + 调用参数：发送加速度计 陀螺仪 磁力计 的原始数据
----------------------------------------------------------*/
void DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
{
    u8 _cnt=0;
    vs16 _temp;

    data_to_send[_cnt++]=title1_send;
    data_to_send[_cnt++]=title2_send;
    data_to_send[_cnt++]=0x02;
    data_to_send[_cnt++]=0;

    _temp = a_x;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = a_y;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = a_z;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    _temp = g_x;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = g_y;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = g_z;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    _temp = m_x;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = m_y;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = m_z;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = 0;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    data_to_send[3] = _cnt-4;

    u8 sum = 0;
    for(u8 i=0; i<_cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;

    DT_Send_Data(data_to_send, _cnt);
}

/*----------------------------------------------------------
 + 实现功能：发送高度传感器信息
 + 调用参数：发送气压计高度 超声波高度 发送单位厘米
----------------------------------------------------------*/
void DT_Send_Senser2(s32 bar_alt,u16 csb_alt)
{
    u8 _cnt=0;

    data_to_send[_cnt++]=title1_send;
    data_to_send[_cnt++]=title2_send;
    data_to_send[_cnt++]=0x07;
    data_to_send[_cnt++]=0;

    data_to_send[_cnt++]=BYTE3(bar_alt);
    data_to_send[_cnt++]=BYTE2(bar_alt);
    data_to_send[_cnt++]=BYTE1(bar_alt);
    data_to_send[_cnt++]=BYTE0(bar_alt);

    data_to_send[_cnt++]=BYTE1(csb_alt);
    data_to_send[_cnt++]=BYTE0(csb_alt);

    data_to_send[3] = _cnt-4;

    u8 sum = 0;
    for(u8 i=0; i<_cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;

    DT_Send_Data(data_to_send, _cnt);
}

/*----------------------------------------------------------
 + 实现功能：发送飞控收到遥控量信息
 + 调用参数：发送飞控收到遥控量数据
----------------------------------------------------------*/
void DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
    u8 _cnt=0;

    data_to_send[_cnt++]=title1_send;
    data_to_send[_cnt++]=title2_send;
    data_to_send[_cnt++]=0x03;
    data_to_send[_cnt++]=0;

    data_to_send[_cnt++]=BYTE1(thr);
    data_to_send[_cnt++]=BYTE0(thr);
    data_to_send[_cnt++]=BYTE1(yaw);
    data_to_send[_cnt++]=BYTE0(yaw);
    data_to_send[_cnt++]=BYTE1(rol);
    data_to_send[_cnt++]=BYTE0(rol);
    data_to_send[_cnt++]=BYTE1(pit);
    data_to_send[_cnt++]=BYTE0(pit);
    data_to_send[_cnt++]=BYTE1(aux1);
    data_to_send[_cnt++]=BYTE0(aux1);
    data_to_send[_cnt++]=BYTE1(aux2);
    data_to_send[_cnt++]=BYTE0(aux2);
    data_to_send[_cnt++]=BYTE1(aux3);
    data_to_send[_cnt++]=BYTE0(aux3);
    data_to_send[_cnt++]=BYTE1(aux4);
    data_to_send[_cnt++]=BYTE0(aux4);
    data_to_send[_cnt++]=BYTE1(aux5);
    data_to_send[_cnt++]=BYTE0(aux5);
    data_to_send[_cnt++]=BYTE1(aux6);
    data_to_send[_cnt++]=BYTE0(aux6);

    data_to_send[3] = _cnt-4;

    u8 sum = 0;
    for(u8 i=0; i<_cnt; i++)
        sum += data_to_send[i];

    data_to_send[_cnt++]=sum;

    DT_Send_Data(data_to_send, _cnt);
}

/*----------------------------------------------------------
 + 实现功能：发送电压数据,电流数据信息
 + 调用参数：电压数据,电流数据
----------------------------------------------------------*/
void DT_Send_Power(u16 votage, u16 current)
{
    u8 _cnt=0;
    u16 temp;

    data_to_send[_cnt++]=title1_send;
    data_to_send[_cnt++]=title2_send;
    data_to_send[_cnt++]=0x05;
    data_to_send[_cnt++]=0;

    temp = votage;
    data_to_send[_cnt++]=BYTE1(temp);
    data_to_send[_cnt++]=BYTE0(temp);
    temp = current;
    data_to_send[_cnt++]=BYTE1(temp);
    data_to_send[_cnt++]=BYTE0(temp);

    data_to_send[3] = _cnt-4;

    u8 sum = 0;
    for(u8 i=0; i<_cnt; i++)
        sum += data_to_send[i];

    data_to_send[_cnt++]=sum;

    DT_Send_Data(data_to_send, _cnt);
}

/*----------------------------------------------------------
 + 实现功能：发送飞控对电机控制量信息
 + 调用参数：发送飞控对电机控制量数据
----------------------------------------------------------*/
void DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
{
    u8 _cnt=0;

    data_to_send[_cnt++]=title1_send;
    data_to_send[_cnt++]=title2_send;
    data_to_send[_cnt++]=0x06;
    data_to_send[_cnt++]=0;

    data_to_send[_cnt++]=BYTE1(m_1);
    data_to_send[_cnt++]=BYTE0(m_1);
    data_to_send[_cnt++]=BYTE1(m_2);
    data_to_send[_cnt++]=BYTE0(m_2);
    data_to_send[_cnt++]=BYTE1(m_3);
    data_to_send[_cnt++]=BYTE0(m_3);
    data_to_send[_cnt++]=BYTE1(m_4);
    data_to_send[_cnt++]=BYTE0(m_4);
    data_to_send[_cnt++]=BYTE1(m_5);
    data_to_send[_cnt++]=BYTE0(m_5);
    data_to_send[_cnt++]=BYTE1(m_6);
    data_to_send[_cnt++]=BYTE0(m_6);
    data_to_send[_cnt++]=BYTE1(m_7);
    data_to_send[_cnt++]=BYTE0(m_7);
    data_to_send[_cnt++]=BYTE1(m_8);
    data_to_send[_cnt++]=BYTE0(m_8);

    data_to_send[3] = _cnt-4;

    u8 sum = 0;
    for(u8 i=0; i<_cnt; i++)
        sum += data_to_send[i];

    data_to_send[_cnt++]=sum;

    DT_Send_Data(data_to_send, _cnt);
}

/*----------------------------------------------------------
 + 实现功能：发送PID信息
 + 调用参数：发送PID数据 放大倍率10E3
----------------------------------------------------------*/
void DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
    u8 _cnt=0;
    vs16 _temp;

    data_to_send[_cnt++]=title1_send;
    data_to_send[_cnt++]=title2_send;
    data_to_send[_cnt++]=0x10+group-1;
    data_to_send[_cnt++]=0;


    _temp = p1_p  * 1000;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = p1_i  * 1000;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = p1_d  * 1000;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = p2_p  * 1000;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = p2_i  * 1000;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = p2_d  * 1000;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = p3_p  * 1000;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = p3_i  * 1000;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = p3_d  * 1000;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    data_to_send[3] = _cnt-4;

    u8 sum = 0;
    for(u8 i=0; i<_cnt; i++)
        sum += data_to_send[i];

    data_to_send[_cnt++]=sum;

    DT_Send_Data(data_to_send, _cnt);
}

/*----------------------------------------------------------
 + 实现功能：用户自定义发送
----------------------------------------------------------*/
void DT_Send_User(s16 a,s16 b,s16 c,s16 d,s16 e,s16 f,s16 g,s16 h,s16 i,s16 j,s16 k,s16 l,s16 m,s16 n,s16 o,s16 p)
{
    u8 _cnt=0;
    vs16 _temp;

    data_to_send[_cnt++]=title1_send;
    data_to_send[_cnt++]=title2_send;
    data_to_send[_cnt++]=0xf1; //用户定义功能字
    data_to_send[_cnt++]=0;

    _temp = a;           //1
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    _temp = b;           //1
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    _temp = c;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    _temp = d;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = e;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = f;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = h;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = i;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = j;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = k;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = l;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = n;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = o;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = p;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    data_to_send[3] = _cnt-4;

    u8 sum = 0;
    for(u8 i=0; i<_cnt; i++)
        sum += data_to_send[i];

    data_to_send[_cnt++]=sum;

    DT_Send_Data(data_to_send, _cnt);
}

/*----------------------------------------------------------
 + 实现功能：由任务调度调用周期1ms
----------------------------------------------------------*/
void Call_Data_transfer(void)
{
#ifdef DT_USE_LIT_SP
		/*light发送速度*/
    /* 定义局部静态变量控制发送周期 */
    static int cnt = 0;
    /* cnt是从1到10000的数据 */
    if(++cnt>10000) cnt = 1;
    /* 1发送姿态数据，周期49ms */
    if((cnt % 49) == 0)
        f.send_status = 1;
    /* 2发送速度数据，周期199ms */
    if((cnt % 199) == 0)
        f.send_speed = 1;
    /* 3发送遥控量数据，周期248ms */
    if((cnt % 248) == 0)
        f.send_rcdata = 1;
    /* 4发送遥控量数据，周期249ms */
    if((cnt % 249) == 0)
        f.send_motopwm = 1;
    /* 5发送传感器数据，周期298ms */
    if((cnt % 298) == 0)
        f.send_senser = 1;
    /* 6发送高度数据，周期399ms */
    if((cnt % 399) == 0)
        f.send_senser2 = 1;
    /* 7发送位置数据，周期499ms */
    if((cnt % 499) == 0)
        f.send_location = 1;
    /* 8发送电压数据，周期998ms */
    if((cnt % 998) == 0)
        f.send_power = 1;
    /* 9发送用户数据，周期999ms */
    if((cnt % 999) == 0)
        f.send_user = 1;
		/*light发送速度*/
#endif
		
#ifdef DT_USE_ATO_SP	
    /*匿名发送速度*/
		static u8 cnt = 0;
		static u8 senser_cnt 	= 10;
		static u8 senser2_cnt = 50;
		static u8 user_cnt 	  = 10;
		static u8 status_cnt 	= 15;
		static u8 rcdata_cnt 	= 20;
		static u8 motopwm_cnt	= 20;
		static u8 power_cnt		=	50;
		static u8 speed_cnt   = 50;
		static u8 location_cnt   = 200;
		
		if((cnt % senser_cnt) == (senser_cnt-1))
			f.send_senser = 1;

		if((cnt % senser2_cnt) == (senser2_cnt-1))
			f.send_senser2 = 1;	

		if((cnt % user_cnt) == (user_cnt-2))
			f.send_user = 1;
		
		if((cnt % status_cnt) == (status_cnt-1))
			f.send_status = 1;	
		
		if((cnt % rcdata_cnt) == (rcdata_cnt-1))
			f.send_rcdata = 1;	
		
		if((cnt % motopwm_cnt) == (motopwm_cnt-2))
			f.send_motopwm = 1;	
		
		if((cnt % power_cnt) == (power_cnt-2))
			f.send_power = 1;		
		
		if((cnt % speed_cnt) == (speed_cnt-3))
			f.send_speed = 1;		
		
		if((cnt % location_cnt) == (location_cnt-3))
		{
			f.send_location += 1;		
		}
		
		if(++cnt>200) cnt = 0;
		/*匿名发送速度*/
#endif		
		
    /* 1发送姿态数据，周期49ms */
    if(f.send_status)
    {
        f.send_status = 0;
        /* 横滚、俯仰、航向、气压cm高度、控制高度模式、解锁状态 */
        DT_Send_Status(IMU_Roll,IMU_Pitch,IMU_Yaw,(0.1f *baro_height),height_ctrl_mode,unlocked_to_fly);
    }
    /* 2发送速度数据，周期199ms */
    else if(f.send_speed)
    {
        f.send_speed = 0;
        #if 0
        /* 调用参数：向北速度 向西速度 向上速度 单位毫米每秒 -> 上位机显示厘米每秒 */
        DT_Send_Speed(0.1f*north_speed,0.1f*west_speed,0.1f*wz_speed);
        #else
        /* X像素速度 Y像素速度 光流像素要结合高度才能算出速度 百分比画面不变量 */
        DT_Send_Speed(Optical_flow_x, Optical_flow_y,0.1f*wz_speed);
        #endif
    }
    /* 3发送遥控量数据 */
    else if(f.send_rcdata)
    {
        f.send_rcdata = 0;
        /* 发送飞控收到遥控量数据 */
        DT_Send_RCData(CH[2]+1500,CH[3]+1500,CH[0]+1500,CH[1]+1500, \
                       CH[4]+1500,CH[5]+1500,CH[6]+1500,CH[7]+1500,0,0);
    }
    /* 4发送遥控量数据 */
    else if(f.send_motopwm)
    {
        f.send_motopwm = 0;
        /* 发送飞控对电机控制量数据 */
        if(PWM_Mode==0)DT_Send_MotoPWM(motor[0],motor[1],motor[2],motor[3],motor[4],motor[5],motor[6],motor[7]);
        else if(PWM_Mode==2)DT_Send_MotoPWM(1000,1000,1000,1000,1000,1000,1000,1000);
        else if(PWM_Mode==1)DT_Send_MotoPWM(0,0,0,0,0,0,0,0);
    }
    /* 5发送传感器数据 */
    else if(f.send_senser)
    {
        f.send_senser = 0;
        /* 发送加速度计 陀螺仪 磁力计 的原始数据 */
        DT_Send_Senser(mpu6050.Acc.x,mpu6050.Acc.y,mpu6050.Acc.z, \
                       mpu6050.Gyro.x,mpu6050.Gyro.y,mpu6050.Gyro.z, \
                       ak8975.Mag_Val.x,ak8975.Mag_Val.y,ak8975.Mag_Val.z);
			
//		/* 占用该通道发送自定义数据change by zhangshuo */
//        DT_Send_Senser(-ctrl_angular_velocity.out.x,ctrl_angular_velocity.out.x,ctrl_angular_velocity.out.x, \
//                       mpu6050.Gyro_deg.x*4,mpu6050.Gyro_deg.y*4,mpu6050.Gyro_deg.z*4, \
//                       -ctrl_attitude.out.x,-ctrl_attitude.out.y,-ctrl_attitude.out.z);
			
			
    }
    /* 6发送高度数据 */
    else if(f.send_senser2)
    {
        f.send_senser2 = 0;
        /* 发送气压计高度 超声波高度 发送单位厘米 */
        DT_Send_Senser2(baro_height*0.1f,ultra_distance/10);
    }
    /* 7发送位置数据，周期499ms */
    else if(f.send_location)
    {
        f.send_location = 0;
        /* 可见卫星数、经度坐标、纬度坐标、悬停飞行模式 其中坐标信息 放大倍率10E5  */
        DT_Send_Location(gpsx.fixmode,gpsx.svnum,t_longitude,t_latitude,position_ctrl_mode);
    }
    /* 8发送电压数据 */
    else if(f.send_power)
    {
        f.send_power = 0;
        /* 电压数据使用 电流数据 并没有被使用 */
        DT_Send_Power(Electric_quantity,456);
    }

    else if(f.send_pid1)
    {
        f.send_pid1 = 0;
        /* 3组PID数据 1、2、3 */
        DT_Send_PID(1,ctrl_angular_velocity.PID[PIDROLL].kp,ctrl_angular_velocity.PID[PIDROLL].ki,ctrl_angular_velocity.PID[PIDROLL].kd, \
                    ctrl_angular_velocity.PID[PIDPITCH].kp,ctrl_angular_velocity.PID[PIDPITCH].ki,ctrl_angular_velocity.PID[PIDPITCH].kd, \
                    ctrl_angular_velocity.PID[PIDYAW].kp,ctrl_angular_velocity.PID[PIDYAW].ki,ctrl_angular_velocity.PID[PIDYAW].kd);
    }

    else if(f.send_pid2)
    {
        f.send_pid2 = 0;
        /* 3组PID数据 4、5、6 */
        DT_Send_PID(2,ctrl_attitude.PID[PIDROLL].kp,ctrl_attitude.PID[PIDROLL].ki,ctrl_attitude.PID[PIDROLL].kd, \
                    ctrl_attitude.PID[PIDPITCH].kp,ctrl_attitude.PID[PIDPITCH].ki,ctrl_attitude.PID[PIDPITCH].kd, \
                    ctrl_attitude.PID[PIDYAW].kp,ctrl_attitude.PID[PIDYAW].ki,ctrl_attitude.PID[PIDYAW].kd);
    }

    else if(f.send_pid3)
    {
        f.send_pid3 = 0;
        /* 3组PID数据 7、8、9 */
        DT_Send_PID(3,pid_setup.groups.hc_sp.kp,pid_setup.groups.hc_sp.ki,pid_setup.groups.hc_sp.kd, \
                    pid_setup.groups.hc_height.kp,pid_setup.groups.hc_height.ki,pid_setup.groups.hc_height.kd, \
                    pid_setup.groups.ctrl3.kp,pid_setup.groups.ctrl3.ki,pid_setup.groups.ctrl3.kd);
    }
    else if(f.send_pid4)
    {
        f.send_pid4 = 0;
        /* 3组PID数据 10 */
        DT_Send_PID(4,pid_setup.groups.ctrl4.kp,pid_setup.groups.ctrl4.ki,pid_setup.groups.ctrl4.kd,
                    pid_setup.groups.ctrl5.kp,pid_setup.groups.ctrl5.ki,pid_setup.groups.ctrl5.kd,
                    pid_setup.groups.ctrl6.kp,pid_setup.groups.ctrl6.ki,pid_setup.groups.ctrl6.kd);
    }

    else if(f.send_user)
    {
        f.send_user = 0;
        /* 用户自定义发送 */
        DT_Send_User(Speed_Front,Speed_Left,Dist_Front,Dist_Left,expect_speed_Front,expect_speed_Left,expect_angle_pitch,expect_angle_roll,north_speed,west_speed,Flow_Comp_x,Flow_Comp_y,OF_Offset_x,OF_Offset_y,expect_of_roll_out,expect_of_pitch_out);
    }
#ifdef DT_USE_USB_HID
	Usb_Hid_Send();
#endif	
	
}

/*----------------------------------------------------------
 + 实现功能：数传初始化
----------------------------------------------------------*/
void Data_transfer_init()
{
    /*----------------------------------------------------------
     + 实现功能：串口2初始化
     + 调用参数功能：
     - u32 bound：波特率 115200
     - u8 Priority：中断主优先级 2
     - u8 SubPriority：中断从优先级 0
     - FunctionalState TXenable：发送中断使能 失能
     - FunctionalState RXenable：就收中断使能 使能
    ----------------------------------------------------------*/
    Device_Usart2_ENABLE_Init(115200,2,0,DISABLE,ENABLE);
	/*飞控usb接口的hid初始化*/
	Usb_Hid_Init();
}

