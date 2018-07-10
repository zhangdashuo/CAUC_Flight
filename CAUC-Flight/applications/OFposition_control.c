/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file OFposition_control.c
*@version V1.0
*@date  2018/5/24
*@brief 光流控制文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#include "OFposition_control.h"
#include "Optical_Flow.h"
#include "mymath.h"
#include "rc.h"
#include "database.h"
#include "driver_parameter.h"
#include "driver_ultrasonic.h"

/* 光流定点最大速度控制 单位 mm/s */
#define Max_OF_SPEED 	 1000.0f

/* 期望的row速度和yaw速度*/
float exp_row_speed = 0;
float exp_yaw_speed = 0;
/* 期望的row速度和yaw速度偏差*/
float exp_row_speed_err , exp_yaw_speed_err;
/* 期望的row速度和yaw速度上一次偏差*/
float exp_row_speed_err_old , exp_yaw_speed_err_old;
/* 期望速度的比例 */
float expect_speed_row_P , expect_speed_yaw_P;
/* 期望速度的积分 */
float expect_speed_row_I , expect_speed_yaw_I;
/* 期望速度的微分 */
float expect_speed_row_D , expect_speed_yaw_D;
/* PID计算的角度值单位0.1mm/s */
float expect_of_pitch,expect_of_roll;
/* 实际输出的角度值 */
float expect_of_pitch_out,expect_of_roll_out;


/*----------------------------------------------------------
 + 实现功能：串级位置PID控制悬停
 + 调用参数：两次调用时间间隔
----------------------------------------------------------*/
void OF_PositionControl_Mode(float Timer_t)
{
	
}

/*----------------------------------------------------------
 + 实现功能：速度PID控制悬停
 + 调用参数：两次调用时间间隔
----------------------------------------------------------*/
void OF_SpeedControl_Mode(float Timer_t)
{
	/* 当超声波高度大于200mm开始光流定点（当前处于调试阶段，后期会和GPS融合） */
	if(ultra_distance > 200)
	{
		/* 光流期望X速度 */
		exp_row_speed = Max_OF_SPEED  *( my_deathzoom( ( CH_filter[ROL]) ,30 )/500.0f ) ;
		/* 光流期望Y速度 */
		exp_yaw_speed = Max_OF_SPEED  *( my_deathzoom( (-CH_filter[PIT]) ,30 )/500.0f );
		/* 光流期望X速度偏差 */
		exp_row_speed_err = exp_row_speed - Flow_Comp_x;
		/* 光流期望Y速度偏差 */
		exp_yaw_speed_err = exp_yaw_speed + Flow_Comp_y;
		

		/* 光流速度偏移量 的比例  */
		expect_speed_row_P=pid_setup.groups.ctrl6.kp *exp_row_speed_err;
		expect_speed_yaw_P=pid_setup.groups.ctrl6.kp *exp_yaw_speed_err;

		/* 光流速度偏移量 的积分  */
		expect_speed_row_I+=pid_setup.groups.ctrl6.ki *exp_row_speed_err;
		expect_speed_yaw_I+=pid_setup.groups.ctrl6.ki *exp_yaw_speed_err;

		/* 光流速度偏移量 的微分  */
		expect_speed_row_D=pid_setup.groups.ctrl6.kd *(exp_row_speed_err-exp_row_speed_err_old);
		expect_speed_yaw_D=pid_setup.groups.ctrl6.kd *(exp_yaw_speed_err-exp_yaw_speed_err_old);
		/* 保存光流速度偏移量 的微分  */
		exp_row_speed_err_old=exp_row_speed_err;
		exp_yaw_speed_err_old=exp_yaw_speed_err;

		/* 低油门时 */
		if(CH_filter[THR]<-400)
		{
			expect_speed_row_I=0;//积分清零
			expect_speed_yaw_I=0;//积分清零
			expect_of_roll_out = 0;//光流计算的角度偏移清零
			expect_of_pitch = 0;
			expect_of_roll_out = 0;//光流计算的输出角度偏移清零
			expect_of_pitch_out = 0;
		}
		expect_speed_row_I = LIMIT(expect_speed_row_I,-300,300);
		expect_speed_yaw_I = LIMIT(expect_speed_yaw_I,-300,300);

		/* 期望速度的PID输出 */
		expect_of_roll=LIMIT(expect_speed_row_P+expect_speed_row_I+expect_speed_row_D,-600,600);
		expect_of_pitch=LIMIT(expect_speed_yaw_P+expect_speed_yaw_I+expect_speed_yaw_D,-600,600);
		
		expect_of_roll_out = expect_of_roll * 0.01f;
		expect_of_pitch_out = expect_of_pitch * 0.01f;
	}
	else
	{
		expect_speed_row_I=0;//积分清零
		expect_speed_yaw_I=0;//积分清零
		
		expect_of_roll_out = 0;//光流计算的补偿角清零
		expect_of_pitch = 0;
		expect_of_roll_out = 0;//光流计算的输出补偿角清零
		expect_of_pitch_out = 0;
	}
	
}
