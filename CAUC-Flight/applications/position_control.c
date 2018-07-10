/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file position_control.c
*@version V1.0
*@date  2018/5/24
*@brief GPS位置控制文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#include "position_control.h"
#include "rc.h"
#include "driver_mpu6050.h"
#include "imu.h"
#include "height_ctrl.h"
#include "arm_math.h"
#include "math.h"
#include "mymath.h"
#include "filter.h"

/* 计算从地球的中心到它的表面的距离 WGS-84常量宏定义 */
#define NAV_FLATTENING		(1.0 / 298.257223563)
#define NAV_E_2			(NAV_FLATTENING * (2.0 - NAV_FLATTENING))
#define Double_PI			3.141592654

/* 计算从地球的中心到它的表面的距离 WGS-84变量 */
double sinLat2,navUkfData_r1,navUkfData_r2;

/* 悬停经纬度 坐标 单位 放大率10E7 */
int STOP_longitude,STOP_latitude;
/* 悬停经纬度 方向 */
u8 STOP_nshemi,STOP_ewhemi;
/* 当前位置 单位 放大率10E7 */
int t_longitude,t_latitude;
/* 方位 速度 放大率10E3 单位 毫米每秒 */
int speed_longitude,speed_latitude;
/* 飞行器 速度 单位毫米每秒 */
float Speed_Front,Speed_Left;
/* 记录为上次数据 */
int d_longitude,d_latitude;
/* 期望与实际偏移量 单位毫米 */
int north_dist,west_dist;
float Dist_Front,Dist_Left;

/* 位置转换期望速度 PID控制变量 */
float expect_dist_Front,expect_dist_Left;
float expect_dist_Front_old,expect_dist_Left_old;
float expect_speed_Front_P,expect_speed_Left_P;
float expect_speed_Front_I,expect_speed_Left_I;
float expect_speed_Front_D,expect_speed_Left_D;
float expect_speed_Front,expect_speed_Left;//

/* 速度转换期望角度 PID控制变量 */
float expect_speed_pitch,expect_speed_roll;
float expect_speed_pitch_old,expect_speed_roll_old;
float expect_speed_pitch_P,expect_speed_roll_P;
float expect_speed_pitch_I,expect_speed_roll_I;
float expect_speed_pitch_D,expect_speed_roll_D;
float expect_angle_pitch,expect_angle_roll;

u8 Get_Initial_PositionFlag = 0;//标记初始位置 

/*
GPS悬停一定确认满足以下条件，在rc.c文件后缀开启
1、GPS位置信息准确，足够开阔
2、能平稳飞行，罗盘数据准确，不自旋
3、PID参数必须正确调整
*/

/*
GPS定点需要用到的参数：

//位置速率
pid_setup.groups.ctrl3.kp, float类型 初始值0.000   速度差 单位毫米每秒 转换为 姿态 单位角度 调整量 0.01倍 的比例
pid_setup.groups.ctrl3.ki, float类型 初始值0.000   速度差 单位毫米每秒 转换为 姿态 单位角度 调整量 0.01倍 的微分
pid_setup.groups.ctrl3.kd, float类型 初始值0.000   速度差 单位毫米每秒 转换为 姿态 单位角度 调整量 0.01倍 的积分
比例的样例值2.000，换算到米就是：
速度偏移1米/s调整姿态是20角度

//位置保持
pid_setup.groups.ctrl4.kp, float类型 初始值0.000   位置差 单位毫米 转换为 速度差 单位毫米每秒 调整量的比例
pid_setup.groups.ctrl4.ki, float类型 初始值0.000   位置差 单位毫米 转换为 速度差 单位毫米每秒 调整量的微分
pid_setup.groups.ctrl4.kd, float类型 初始值0.000   位置差 单位毫米 转换为 速度差 单位毫米每秒 调整量的积分
比例的样例值1.500，换算到米就是：
位置偏移1米调整速度是1.5m/s
*/

/*----------------------------------------------------------
 + 实现功能：计算从地球的中心到它的表面的距离
 + 调用参数：纬度
----------------------------------------------------------*/
void navUkfCalcEarthRadius(double lat)
{
    float temp_f32;
    sinLat2 = arm_sin_f32(lat * Double_PI / 180.0);
    sinLat2 = sinLat2 * sinLat2;
    arm_sqrt_f32( 1.0 - (NAV_E_2 * sinLat2) , &temp_f32);
    navUkfData_r1 = (6378137.0  * Double_PI / 180.0) *
                    (1.0 - NAV_E_2) / pow(1.0 - (NAV_E_2 * sinLat2), (3.0 / 2.0));
    navUkfData_r2 = (6378137.0  * Double_PI / 180.0) /
                    temp_f32 * arm_cos_f32(lat * Double_PI / 180.0);
}

/*----------------------------------------------------------
 + 实现功能：串级PID控制悬停
 + 调用参数：两次调用时间间隔
----------------------------------------------------------*/
void PositionControl_Mode(float Timer_t)
{
	static u32 t_longitude_old,t_latitude_old;//保存上次的有效数据，用于滤波
	
    /* 确定位置信息和开启定高模式后方可进入功能 */
    if(gpsx.fixmode >= 2 && height_ctrl_mode >0)
    {
        /* 当前位置经度赋值 单位 放大率10E7 */
        t_longitude=(int)gpsx.longitude;
		if(!t_longitude)//如果这次得到的经度为0则使用上次的值
		{
			t_longitude = t_longitude_old;
		}
		else//如果这次得到的经度不是0则保存
		{
			t_longitude_old = t_longitude;
		}
        /* 当前位置纬度赋值 单位 放大率10E7 */
        t_latitude=(int)gpsx.latitude;
		if(!t_latitude)//如果这次得到的纬度为0则使用上次的值
		{
			t_latitude = t_latitude_old;
		}
		else
		{
			t_latitude_old = t_latitude;
		}
        /* 当前位置经度中位数滤波 单位 放大率10E7 */
        t_longitude = Moving_Median_int(0,5,t_longitude);
        /* 当前位置纬度中位数滤波 单位 放大率10E7 */
        t_latitude = Moving_Median_int(1,5,t_latitude);

        /* 计算从地球的中心到它的表面的距离 */
        navUkfCalcEarthRadius((double)t_latitude / 10000000.0);

        /* 自东向西方向速度 放大率10E3 单位 毫米每秒 */
        speed_longitude =  navUkfData_r2 * (double)(d_longitude - t_longitude) / 10000.0;
		if( speed_longitude > 5000 || speed_longitude < -5000)
			speed_longitude = 0;
        /* 自南向北方向速度 放大率10E3 单位 毫米每秒 */
        speed_latitude = navUkfData_r1 * (double)(t_latitude - d_latitude) / 10000.0 ;
		if(speed_latitude > 5000 || speed_latitude < -5000)
			speed_latitude = 0;

        /* 记录为上次数据 */
        d_longitude = t_longitude;
        d_latitude = t_latitude;
		

        /* 当标记初始位置后方可进入悬停模式 */
        if(position_ctrl_mode==3 && Get_Initial_PositionFlag)
        {
			/*
			引用关键变量：由加速度计和GPS融合计算出速度：
				north_speed：向北速度，单位毫米每秒
				west_speed：向西速度，单位毫米每秒
			*/

			/* 飞行器向飞行器的前方速度 单位毫米每秒 */
			Speed_Front = north_speed * arm_cos_f32(IMU_Yaw*3.14159f/180.0f)
						  - west_speed * arm_sin_f32(IMU_Yaw*3.14159f/180.0f);

			/* 飞行器向左速度，单位毫米每秒 */
			Speed_Left = west_speed * arm_cos_f32(IMU_Yaw*3.14159f/180.0f)
						 + north_speed * arm_sin_f32(IMU_Yaw*3.14159f/180.0f);
			
			/* 期望与实际向北偏移量 单位毫米 */
			north_dist = navUkfData_r1 *(double)(t_latitude-STOP_latitude) / 10000.0;
			/* 期望与实际向西偏移量 单位毫米 */
			west_dist = navUkfData_r2 *(double)(STOP_longitude-t_longitude) / 10000.0;

			/* 飞行器位置向飞行器前方偏移量 */
			Dist_Front = north_dist * arm_cos_f32(IMU_Yaw*3.14159f/180.0f)
						 - west_dist * arm_sin_f32(IMU_Yaw*3.14159f/180.0f);

			/* 飞行器位置向飞行器左方偏移量 */
			Dist_Left = west_dist * arm_cos_f32(IMU_Yaw*3.14159f/180.0f)
						+ north_dist * arm_sin_f32(IMU_Yaw*3.14159f/180.0f);
			
            /* 向前期望位置 */
            expect_dist_Front = - Dist_Front ;
            /* 向左期望位置 */
            expect_dist_Left = - Dist_Left ;

            /* 位置差 单位毫米 转换为 速度差 单位毫米每秒 调整量的比例  */
            expect_speed_Front_P=pid_setup.groups.ctrl4.kp *expect_dist_Front;
            expect_speed_Left_P=pid_setup.groups.ctrl4.kp *expect_dist_Left;

            /* 位置差 单位毫米 转换为 速度差 单位毫米每秒 调整量的积分  */
            expect_speed_Front_I+=pid_setup.groups.ctrl4.ki *expect_dist_Front;
            expect_speed_Left_I+=pid_setup.groups.ctrl4.ki *expect_dist_Left;

            /* 位置差 单位毫米 转换为 速度差 单位毫米每秒 调整量的微分  */
            expect_speed_Front_D=pid_setup.groups.ctrl4.kd *(expect_dist_Front-expect_dist_Front_old);
            expect_speed_Left_D=pid_setup.groups.ctrl4.kd *(expect_dist_Left-expect_dist_Left_old);
            expect_dist_Front_old=expect_dist_Front;
            expect_dist_Left_old=expect_dist_Left;

            /* 低油门时 */
            if(CH_filter[THR]<-400)
            {
                expect_speed_Front_I=0;//积分清零
                expect_speed_Left_I=0;//积分清零
            }
            expect_speed_Front_I = LIMIT(expect_speed_Front_I,-500,500);
            expect_speed_Left_I = LIMIT(expect_speed_Left_I,-500,500);

            /* 期望速度的PID输出 */
            expect_speed_Front=LIMIT(expect_speed_Front_P+expect_speed_Front_I+expect_speed_Front_D,-600,600);
            expect_speed_Left=LIMIT(expect_speed_Left_P+expect_speed_Left_I+expect_speed_Left_D,-600,600);

            /* 向前期望速度 减去 向飞行器的前方速度 转换为 单位分米每秒 再匹配PITCH的符号 */
            expect_speed_pitch = - ( expect_speed_Front - Speed_Front)*0.01f;
            expect_speed_roll = - ( expect_speed_Left - Speed_Left)*0.01f;

            /* 速度差 单位分米每秒 转换为 姿态 单位角度 调整量的比例  */
            expect_speed_pitch_P=pid_setup.groups.ctrl3.kp *expect_speed_pitch;
            expect_speed_roll_P=pid_setup.groups.ctrl3.kp *expect_speed_roll;

            /* 速度差 单位分米每秒 转换为 姿态 单位角度 调整量的积分  */
            expect_speed_pitch_I+=pid_setup.groups.ctrl3.ki *expect_speed_pitch;
            expect_speed_roll_I+=pid_setup.groups.ctrl3.ki *expect_speed_roll;

            /* 速度差 单位分米每秒 转换为 姿态 单位角度 调整量的微分  */
            expect_speed_pitch_D=pid_setup.groups.ctrl3.kd *(expect_speed_pitch-expect_speed_pitch_old);
            expect_speed_roll_D=pid_setup.groups.ctrl3.kd *(expect_speed_roll-expect_speed_roll_old);
            expect_speed_pitch_old=expect_speed_pitch;
            expect_speed_roll_old=expect_speed_roll;

            /* 低油门时 */
            if(CH_filter[THR]<-400)
            {
                expect_speed_pitch_I=0;//积分清零
                expect_speed_roll_I=0;//积分清零
            }
            expect_speed_pitch_I = LIMIT(expect_speed_pitch_I,-5,5);
            expect_speed_roll_I = LIMIT(expect_speed_roll_I,-5,5);

            /* 期望姿态的PID输出 */
            expect_angle_pitch=LIMIT(expect_speed_pitch_P+expect_speed_pitch_I+expect_speed_pitch_D,-6,6);
            expect_angle_roll=LIMIT(expect_speed_roll_P+expect_speed_roll_I+expect_speed_roll_D,-6,6);
        }
        /* 记录悬停模式 */
        else if(position_ctrl_mode==2)
        {
			if(t_longitude && t_latitude)//如果经纬度都非零才可以进行标记
			{
				/* 标记初始位置成功 */
				Get_Initial_PositionFlag = 1;
				/* 记录经纬度 */
				STOP_longitude = t_longitude;
				STOP_latitude = t_latitude;
				STOP_ewhemi = gpsx.ewhemi;	//东经/西经,E:东经;W:西经
				STOP_nshemi = gpsx.nshemi;	//北纬/南纬,N:北纬;S:南纬
			}
			else
			{
				/* 不悬停 */
				position_ctrl_mode=1;
			}

            expect_speed_Front_I=0;//积分清零
            expect_speed_Left_I=0;//积分清零
            expect_speed_pitch_I=0;//积分清零
            expect_speed_roll_I=0;//积分清零
        }
		/* 如果没有记录初始位置，则不允许进入GPS定点模式 */
		else
		{
			/* 不悬停 */
			position_ctrl_mode=1;
		}
    }
    /* 突然没有位置信息 */
    else
    {
        /* 不悬停 */
        position_ctrl_mode=1;
    }
}

