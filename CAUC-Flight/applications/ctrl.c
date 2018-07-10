/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file ctrl.c
*@version V1.0
*@date  2018/5/24
*@brief 飞控控制文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#include "stm32f4xx.h"
#include "ctrl.h"
#include "height_ctrl.h"
#include "mymath.h"
#include "position_control.h"
#include "OFposition_control.h"

/* 允许高度控制宏定义 0失能，1使能 */
#define CTRL_HEIGHT 1

/* 遥控器控制量能达到的最大期望角度 */
#define MAX_CTRL_ANGLE			25.0f
/* 角度误差N时，期望角速度达到最大（可以通过调整CTRL_2的P值调整） */
#define ANGLE_TO_MAX_AS 		30.0f
/* 姿态控制积分幅度 */
#define CTRL_2_INT_LIMIT 		0.5f *MAX_CTRL_ANGLE

/* ROL,PIT遥控器控制量能达到的最大期望角速度 */
#define MAX_CTRL_ASPEED 	 	300.0f
/* YAW遥控器控制量能达到的最大期望角速度 */
#define MAX_CTRL_YAW_SPEED 		150.0f
/* 期望角速度控制积分限幅 */
#define CTRL_1_INT_LIMIT 		0.5f *MAX_CTRL_ASPEED

/* 电机转速控制量百分比 */
#define MAX_PWM		  	100			//	最大PWM输出为100%油门
#define MAX_THR       	80 			//	油门通道最大占比80%，留20%给控制量
#define READY_SPEED   	20			//	解锁后电机转速20%油门

/* 机架类型 */
char OP_COPTER =0;

/* 角速度控制结构体 */
ctrl_t ctrl_angular_velocity;
/* 姿态控制结构体 */
ctrl_t ctrl_attitude;
/* 油门赋值 */
float thr_value;
/* 低油门信号判断 */
u8 Thr_Low;
/* 滤波后油门数据 */
float Thr_Weight;

/* 遥控控制期望姿态角度 */
xyz_f_t except_A = {0,0,0};
/* 期望姿态角度偏移 */
xyz_f_t ctrl_angle_offset = {3.5,0,0};
/* 期望角速度 */
xyz_f_t except_AS;

/* 记录的陀螺仪数据 */
float g_old[ITEMS];
/* 单个电机的总控制量 */
float motor[8];

/*----------------------------------------------------------
 + 实现功能：恢复默认控制幅度
----------------------------------------------------------*/
void Ctrl_Para_Init()
{
    /* 微分控制幅度 */
    ctrl_angular_velocity.PID[PIDROLL].kdamp  = 1;
    ctrl_angular_velocity.PID[PIDPITCH].kdamp = 1;
    ctrl_angular_velocity.PID[PIDYAW].kdamp   = 1;
    /* 角速度的偏差权重比例系数 */
    ctrl_angular_velocity.FB = 0.20;
}

/*----------------------------------------------------------
 + 实现功能：油门信号控制
 + 调用参数：两次调用间隔
----------------------------------------------------------*/
void Thr_Ctrl(float T)
{
    /* 油门值 */
    static float thr;
    /* 滤波后的油门值 */
    static float Thr_tmp;

    /* 油门值 范围0 ~ 1000 */
    thr = 500 + CH_filter[THR];
    /* 油门数据积分滤波 */
    Thr_tmp += 10 *3.14f *T *(thr/400.0f - Thr_tmp);
    /* 滤波后油门数据限幅 */
    Thr_Weight = LIMIT(Thr_tmp,0,1);

    /* 低油门信号判断 */
    if( thr < 100 )
        Thr_Low = 1;
    else
        Thr_Low = 0;
    /* 允许高度控制 */
#if(CTRL_HEIGHT)
    /* 油门控制高度 */
    Height_Ctrl(T,thr);
    /* 油门赋值 */
    thr_value = Thr_Weight *height_ctrl_out ;
#else
    /* 实际使用值 */
    thr_value = thr;
#endif
    /* 油门限幅 */
    thr_value = LIMIT(thr_value,0,10 *MAX_THR *MAX_PWM/100);
}

/*----------------------------------------------------------
 + 实现功能：角速度控制量 转换到 电机转速的输出量
 + 调用参数：角速度控制量
----------------------------------------------------------*/
void All_Out(float out_roll,float out_pitch,float out_yaw)
{
    s16 motor_out[8];
    /* 循环计数变量 */
    u8 i;
    /* 姿态作用于电机位置的控制量 */
    float posture_value[8];
    /* 姿态对电机的实际控制量 */
    float curve[8];

    /* 航向的角速度控制量限幅 防止动力不足 */
    out_yaw = LIMIT( out_yaw , -5*MAX_THR ,5*MAX_THR );

    /*默认是四旋翼X模式，需要修改不同机架在代码中改下即可
        CCW是从上方向下看逆时针旋转，CW与CCW刚好相反，注意
    前方偏右是连接1号电机电调，逆时针顺序依次是2开始*/
    /**注意电机旋转顺序,CCW和CW是从正上空方参照向下看的,不是以电机为参照的**/
    /* 单层四旋翼模式 */
    if(OP_COPTER == 0)
    {
        /*         CW2     1CCW           */
        /*             \ /                */
        /*             / \                */
        /*        CCW3     4CW            */
        posture_value[0] = - out_roll + out_pitch + out_yaw ;//右前方，CCW
        posture_value[1] = + out_roll + out_pitch - out_yaw ;//左前方， CW
        posture_value[2] = + out_roll - out_pitch + out_yaw ;//左后方，CCW
        posture_value[3] = - out_roll - out_pitch - out_yaw ;//右后方， CW

        posture_value[4]=posture_value[5]=posture_value[6]=posture_value[7]=0;
    }
    /* 单层六旋翼模式 */
    else if(OP_COPTER == 1)
    {
        /*         CW2     1CCW           */
        /*            \   /               */
        /*             \ /                */
        /*   CCW3—— ——     —— ——6CW       */
        /*             / \                */
        /*            /   \               */
        /*         CW4     5CCW           */
        posture_value[0] = + out_pitch - out_roll/2 + out_yaw;// 1点钟方向，CCW
        posture_value[1] = + out_pitch + out_roll/2 - out_yaw;//11点钟方向， CW
        posture_value[2] =             + out_roll   + out_yaw;// 9点钟方向，CCW
        posture_value[3] = - out_pitch + out_roll/2 - out_yaw;// 7点钟方向， CW
        posture_value[4] = - out_pitch - out_roll/2 + out_yaw;// 5点钟方向，CCW
        posture_value[5] =             - out_roll   - out_yaw;// 3点钟方向， CW
        posture_value[6] = posture_value[7] = 0;
    }
    /* 单层八旋翼模式 */
    else if(OP_COPTER == 2)
    {
        /*         CW1     0CCW           */
        /*            \   /               */
        /*    CCW2     \ /     7CW        */
        /*       —— ——     —— ——          */
        /*     CW3     / \     6CCW       */
        /*            /   \               */
        /*        CCW4     5CW            */
        posture_value[0] = + out_pitch   - out_roll/2 + out_yaw;//约 1点钟方向，CCW
        posture_value[1] = + out_pitch   + out_roll/2 - out_yaw;//约11电阻方向， CW
        posture_value[2] = + out_pitch/2 + out_roll   + out_yaw;//约10点钟方向，CCW
        posture_value[3] = - out_pitch/2 + out_roll   - out_yaw;//约 8点钟方向， CW
        posture_value[4] = - out_pitch   + out_roll/2 + out_yaw;//约 7点钟方向，CCW
        posture_value[5] = - out_pitch   - out_roll/2 - out_yaw;//约 5点钟方向， CW
        posture_value[6] = - out_pitch/2 - out_roll   + out_yaw;//约 4点钟方向，CCW
        posture_value[7] = + out_pitch/2 - out_roll   - out_yaw;//约 2点钟方向， CW
    }
    /* 双层四旋翼模式 */
    else if(OP_COPTER == 3)
    {
        //1,2,3,4上层;5,6,7,8下层,注意电机旋转顺序,
        //CCW和CW是从正上空方参照向下看的,不是以电机为参照的
        /*    CCW6 CW2     1CCW 5CW       */
        /*             \ /                */
        /*             / \                */
        /*    CW7 CCW3     4CW 8CCW       */
        posture_value[0] = - out_roll + out_pitch + out_yaw ;//右前方，CCW
        posture_value[1] = + out_roll + out_pitch - out_yaw ;//左前方， CW
        posture_value[2] = + out_roll - out_pitch + out_yaw ;//左后方，CCW
        posture_value[3] = - out_roll - out_pitch - out_yaw ;//右后方， CW

        posture_value[4] = - out_roll + out_pitch - out_yaw ;//右前方， CW
        posture_value[5] = + out_roll + out_pitch + out_yaw ;//左前方，CCW
        posture_value[6] = + out_roll - out_pitch - out_yaw ;//左后方， CW
        posture_value[7] = - out_roll - out_pitch + out_yaw ;//右后方，CCW
    }

    /* 作用于电机位置的控制量 */
    for(i=0; i<8; i++)
    {
        /* 姿态作用于电机位置的控制量限幅 */
        posture_value[i] = LIMIT(posture_value[i], -1000,1000 );
        /* 姿态对电机的实际控制量 */
        curve[i] = (0.55f + 0.45f *ABS(posture_value[i])/1000.0f) *posture_value[i] ;
        /* 单个电机的总控制量 */
        motor[i] = thr_value + Thr_Weight *curve[i] ;
    }
    /* 已经解锁 */
    if(unlocked_to_fly)
    {
        /* 遥控器控制量的油门拉起 */
        if( !Thr_Low )
            for(i=0; i<8; i++)
                /* 保证大于在电机最小起转转速 */
                motor[i] = LIMIT(motor[i], (10 *READY_SPEED),(10*MAX_PWM) );
        /* 遥控器控制量的油门低 */
        else
            for(i=0; i<8; i++)
                motor[i] = LIMIT(motor[i], 0,(10*MAX_PWM) );
    }
    /* 未解锁 */
    else
        /* 电机停转 */
        for(i=0; i<8; i++)
            motor[i] = 0;

    /* int16到float数据类型转换 */
    for(i=0; i<8; i++)
        motor_out[i] = (s16)(motor[i]);
    /* 调用函数赋值PWM */
    SetPwm(motor_out);
}

/*----------------------------------------------------------
 + 实现功能：姿态PID控制角速度 由任务调度调用周期5ms
 + 调用参数：两次调用间隔
----------------------------------------------------------*/
void CTRL_attitude(float T)
{
    /* 不悬停模式 */
    if(position_ctrl_mode==1)
    {
        /* 遥控器控制量中心死区30 单位 微秒 */
        except_A.x  = expect_of_roll_out + MAX_CTRL_ANGLE  *( my_deathzoom( ( CH_filter[ROL]) ,30 )/500.0f );
        except_A.y  = expect_of_pitch_out + MAX_CTRL_ANGLE  *( my_deathzoom( (-CH_filter[PIT]) ,30 )/500.0f );
        /* 偏差为 姿态自稳光流自稳 与 遥控器控制量 之和 */
        ctrl_attitude.err.x =  To_180_degrees( ctrl_angle_offset.x + except_A.x - IMU_Roll  );
        ctrl_attitude.err.y =  To_180_degrees( ctrl_angle_offset.y + except_A.y - IMU_Pitch );
		
//		ctrl_attitude.err.y = 0;
		
        /* 遥控器控制量的油门拉起 */
        if( Thr_Low == 0 )
            /* 期望航向姿态由遥控器航向控制积分 */
            except_A.z += (s16)( MAX_CTRL_YAW_SPEED *( my_deathzoom_2( (CH_filter[YAW]) ,40 )/500.0f ) ) *T*0.3f ;
        /* 遥控器控制量的油门低 */
        else
            /* 期望航向姿态为当前姿态 */
            except_A.z += 1 *3.14 *T *( IMU_Yaw - except_A.z );
        /* 角度范围控制在+-180角度 */
        except_A.z = To_180_degrees(except_A.z);
        /* 偏差为 姿态自稳 与 遥控器控制量 之和 */
        ctrl_attitude.err.z =  LIMIT(To_180_degrees( ctrl_angle_offset.z + except_A.z - IMU_Yaw ), -45, 45 );	
		
//		ctrl_attitude.err.z = 0;
    }
    else if(position_ctrl_mode==2)//航向锁定模式
    {
        /* 遥控器控制量中心死区30 单位 微秒 */
        except_A.x  = MAX_CTRL_ANGLE  *( my_deathzoom( ( CH_filter[ROL]) ,30 )/500.0f );
        except_A.y  = MAX_CTRL_ANGLE  *( my_deathzoom( (-CH_filter[PIT]) ,30 )/500.0f );
        /* 偏差为 姿态自稳 与 遥控器控制量 之和 */
        ctrl_attitude.err.x =  To_180_degrees( ctrl_angle_offset.x + except_A.x - IMU_Roll  );
        ctrl_attitude.err.y =  To_180_degrees( ctrl_angle_offset.y + except_A.y - IMU_Pitch );
        /* 期望航向姿态为正北 */
        except_A.z  =  90*( my_deathzoom( ( CH_filter[YAW]) ,30 )/500.0f );
        /* 偏差为 姿态自稳 与 遥控器控制量 之和 */
        ctrl_attitude.err.z =  LIMIT( To_180_degrees( ctrl_angle_offset.z + except_A.z - IMU_Yaw ), -45 , 45 );
    }
    /* 悬停模式 */
    else if(position_ctrl_mode==3)
    {
        /* 遥控器控制量中心死区30 单位 微秒 */
        except_A.x  = expect_angle_roll + MAX_CTRL_ANGLE  *( my_deathzoom( ( CH_filter[ROL]) ,30 )/500.0f );
        except_A.y  = expect_angle_pitch + MAX_CTRL_ANGLE  *( my_deathzoom( (-CH_filter[PIT]) ,30 )/500.0f );
        /* 偏差为 姿态自稳 与 遥控器控制量 之和 */
        ctrl_attitude.err.x =  LIMIT( To_180_degrees( ctrl_angle_offset.x + except_A.x - IMU_Roll ) ,-25 , 25 );
        ctrl_attitude.err.y =  LIMIT( To_180_degrees( ctrl_angle_offset.y + except_A.y - IMU_Pitch ) ,-25 , 25 );
        /* 遥控器控制量的油门拉起 */
        if( Thr_Low == 0 )
            /* 期望航向姿态由遥控器航向控制积分 */
            except_A.z += (s16)( MAX_CTRL_YAW_SPEED *( my_deathzoom_2( (CH_filter[YAW]) ,40 )/500.0f ) ) *T*0.3f ;
        /* 遥控器控制量的油门低 */
        else
            /* 期望航向姿态为当前姿态 */
            except_A.z += 1 *3.14 *T *( IMU_Yaw - except_A.z );
        except_A.z = To_180_degrees(except_A.z);
        /* 偏差为 姿态自稳 与 遥控器控制量 之和 */
        ctrl_attitude.err.z =  LIMIT(To_180_degrees( ctrl_angle_offset.z + except_A.z - IMU_Yaw ), -45, 45 );
    }

    /* 计算角度误差权重 */
    ctrl_attitude.err_weight.x = ABS(ctrl_attitude.err.x)/ANGLE_TO_MAX_AS;
    ctrl_attitude.err_weight.y = ABS(ctrl_attitude.err.y)/ANGLE_TO_MAX_AS;
    ctrl_attitude.err_weight.z = ABS(ctrl_attitude.err.z)/ANGLE_TO_MAX_AS;
    /* 角度误差微分（跟随误差曲线变化）*/
    ctrl_attitude.err_d.x = 10 *ctrl_attitude.PID[PIDROLL].kd  *(ctrl_attitude.err.x - ctrl_attitude.err_old.x) *( 0.005f/T ) *( 0.65f + 0.35f *ctrl_attitude.err_weight.x );
    ctrl_attitude.err_d.y = 10 *ctrl_attitude.PID[PIDPITCH].kd *(ctrl_attitude.err.y - ctrl_attitude.err_old.y) *( 0.005f/T ) *( 0.65f + 0.35f *ctrl_attitude.err_weight.y );
    ctrl_attitude.err_d.z = 10 *ctrl_attitude.PID[PIDYAW].kd 	 *(ctrl_attitude.err.z - ctrl_attitude.err_old.z) *( 0.005f/T ) *( 0.65f + 0.35f *ctrl_attitude.err_weight.z );
    /* 角度误差积分 */
    ctrl_attitude.err_i.x += ctrl_attitude.PID[PIDROLL].ki  *ctrl_attitude.err.x *T;
    ctrl_attitude.err_i.y += ctrl_attitude.PID[PIDPITCH].ki *ctrl_attitude.err.y *T;
    ctrl_attitude.err_i.z += ctrl_attitude.PID[PIDYAW].ki   *ctrl_attitude.err.z *T;
	/* 当模式进行切换时将积分全部清零 */
	if( position_ctrl_mode_old != position_ctrl_mode)
	{
		ctrl_attitude.err_i.x = 0;
		ctrl_attitude.err_i.y = 0;
		ctrl_attitude.err_i.z = 0;
	}
    /* 角度误差积分分离 */
    ctrl_attitude.eliminate_I.x = Thr_Weight *CTRL_2_INT_LIMIT;
    ctrl_attitude.eliminate_I.y = Thr_Weight *CTRL_2_INT_LIMIT;
    ctrl_attitude.eliminate_I.z = Thr_Weight *CTRL_2_INT_LIMIT;
    /* 角度误差积分限幅 */
    ctrl_attitude.err_i.x = LIMIT( ctrl_attitude.err_i.x, -ctrl_attitude.eliminate_I.x,ctrl_attitude.eliminate_I.x );
    ctrl_attitude.err_i.y = LIMIT( ctrl_attitude.err_i.y, -ctrl_attitude.eliminate_I.y,ctrl_attitude.eliminate_I.y );
    ctrl_attitude.err_i.z = LIMIT( ctrl_attitude.err_i.z, -ctrl_attitude.eliminate_I.z,ctrl_attitude.eliminate_I.z );
    /* 对用于计算比例项输出的角度误差限幅 */
    ctrl_attitude.err.x = LIMIT( ctrl_attitude.err.x, -90, 90 );
    ctrl_attitude.err.y = LIMIT( ctrl_attitude.err.y, -90, 90 );
    ctrl_attitude.err.z = LIMIT( ctrl_attitude.err.z, -90, 90 );
    /* 角度PID输出 */
    ctrl_attitude.out.x = ctrl_attitude.PID[PIDROLL].kp  *( ctrl_attitude.err.x + ctrl_attitude.err_d.x + ctrl_attitude.err_i.x );
    ctrl_attitude.out.y = ctrl_attitude.PID[PIDPITCH].kp *( ctrl_attitude.err.y + ctrl_attitude.err_d.y + ctrl_attitude.err_i.y );
    ctrl_attitude.out.z = ctrl_attitude.PID[PIDYAW].kp   *( ctrl_attitude.err.z + ctrl_attitude.err_d.z + ctrl_attitude.err_i.z );
    /* 记录历史数据 */
    ctrl_attitude.err_old.x = ctrl_attitude.err.x;
    ctrl_attitude.err_old.y = ctrl_attitude.err.y;
    ctrl_attitude.err_old.z = ctrl_attitude.err.z;
	/* 保存当前的模式 */
	position_ctrl_mode_old = position_ctrl_mode;
}

/*----------------------------------------------------------
 + 实现功能：角速度电机输出量 由任务调度调用周期2ms
 + 调用参数：两次调用间隔
----------------------------------------------------------*/
void CTRL_angular_velocity(float T)
{
    /* 期望角速度 */
    xyz_f_t EXP_LPF_TMP;
	
    /* 期望角速度 */	
    EXP_LPF_TMP.x = MAX_CTRL_ASPEED *(ctrl_attitude.out.x/ANGLE_TO_MAX_AS);
    EXP_LPF_TMP.y = MAX_CTRL_ASPEED *(ctrl_attitude.out.y/ANGLE_TO_MAX_AS);
    EXP_LPF_TMP.z = MAX_CTRL_ASPEED *(ctrl_attitude.out.z/ANGLE_TO_MAX_AS);
    /* 期望角速度 */
    except_AS.x = EXP_LPF_TMP.x;
    except_AS.y = EXP_LPF_TMP.y;
    except_AS.z = EXP_LPF_TMP.z;
    except_AS.x = LIMIT(except_AS.x, -MAX_CTRL_ASPEED,MAX_CTRL_ASPEED );
    except_AS.y = LIMIT(except_AS.y, -MAX_CTRL_ASPEED,MAX_CTRL_ASPEED );
    except_AS.z = LIMIT(except_AS.z, -MAX_CTRL_ASPEED,MAX_CTRL_ASPEED );
    /* 角速度直接微分（角加速度），负反馈可形成角速度的阻尼（阻碍角速度的变化）*/
    ctrl_angular_velocity.damp.x = ( mpu6050.Gyro_deg.x - g_old[A_X]) *( 0.002f/T );
    ctrl_angular_velocity.damp.y = (-mpu6050.Gyro_deg.y - g_old[A_Y]) *( 0.002f/T );
    ctrl_angular_velocity.damp.z = (-mpu6050.Gyro_deg.z - g_old[A_Z]) *( 0.002f/T );
    /* 角速度偏差 */
    ctrl_angular_velocity.err.x =  ( except_AS.x - mpu6050.Gyro_deg.x ) *(300.0f/MAX_CTRL_ASPEED);
    ctrl_angular_velocity.err.y =  ( except_AS.y + mpu6050.Gyro_deg.y ) *(300.0f/MAX_CTRL_ASPEED);
    ctrl_angular_velocity.err.z =  ( except_AS.z + mpu6050.Gyro_deg.z ) *(300.0f/MAX_CTRL_ASPEED);
    /* 角速度偏差权重 */
    ctrl_angular_velocity.err_weight.x = ABS(ctrl_angular_velocity.err.x)/MAX_CTRL_ASPEED;
    ctrl_angular_velocity.err_weight.y = ABS(ctrl_angular_velocity.err.y)/MAX_CTRL_ASPEED;
    ctrl_angular_velocity.err_weight.z = ABS(ctrl_angular_velocity.err.z)/MAX_CTRL_YAW_SPEED;
    /* 角速度微分 */
    ctrl_angular_velocity.err_d.x = ( ctrl_angular_velocity.PID[PIDROLL].kd  *( -10 *ctrl_angular_velocity.damp.x) *( 0.002f/T ) );
    ctrl_angular_velocity.err_d.y = ( ctrl_angular_velocity.PID[PIDPITCH].kd *( -10 *ctrl_angular_velocity.damp.y) *( 0.002f/T ) );
    ctrl_angular_velocity.err_d.z = ( ctrl_angular_velocity.PID[PIDYAW].kd   *( -10 *ctrl_angular_velocity.damp.z) *( 0.002f/T ) );
    /* 角速度误差积分 */
    ctrl_angular_velocity.err_i.x += ctrl_angular_velocity.PID[PIDROLL].ki  *(ctrl_angular_velocity.err.x - ctrl_angular_velocity.damp.x) *T;
    ctrl_angular_velocity.err_i.y += ctrl_angular_velocity.PID[PIDPITCH].ki *(ctrl_angular_velocity.err.y - ctrl_angular_velocity.damp.y) *T;
    ctrl_angular_velocity.err_i.z += ctrl_angular_velocity.PID[PIDYAW].ki 	*(ctrl_angular_velocity.err.z - ctrl_angular_velocity.damp.z) *T;
    /* 角速度误差积分分离 */
    ctrl_angular_velocity.eliminate_I.x = Thr_Weight *CTRL_1_INT_LIMIT ;
    ctrl_angular_velocity.eliminate_I.y = Thr_Weight *CTRL_1_INT_LIMIT ;
    ctrl_angular_velocity.eliminate_I.z = Thr_Weight *CTRL_1_INT_LIMIT ;
    /* 角速度误差积分限幅 */
    ctrl_angular_velocity.err_i.x = LIMIT( ctrl_angular_velocity.err_i.x, -ctrl_angular_velocity.eliminate_I.x,ctrl_angular_velocity.eliminate_I.x );
    ctrl_angular_velocity.err_i.y = LIMIT( ctrl_angular_velocity.err_i.y, -ctrl_angular_velocity.eliminate_I.y,ctrl_angular_velocity.eliminate_I.y );
    ctrl_angular_velocity.err_i.z = LIMIT( ctrl_angular_velocity.err_i.z, -ctrl_angular_velocity.eliminate_I.z,ctrl_angular_velocity.eliminate_I.z );
    /* 角速度PID输出 */
    ctrl_angular_velocity.out.x = 2 *( ctrl_angular_velocity.FB *LIMIT((0.45f + 0.55f*ctrl_attitude.err_weight.x),0,1)*except_AS.x + \
																		   ( 1 - ctrl_angular_velocity.FB ) *ctrl_angular_velocity.PID[PIDROLL].kp  *( ctrl_angular_velocity.err.x + ctrl_angular_velocity.err_d.x + ctrl_angular_velocity.err_i.x ) \
																		 );
    ctrl_angular_velocity.out.y = 2 *( ctrl_angular_velocity.FB *LIMIT((0.45f + 0.55f*ctrl_attitude.err_weight.y),0,1)*except_AS.y + \
																			 ( 1 - ctrl_angular_velocity.FB ) *ctrl_angular_velocity.PID[PIDPITCH].kp *( ctrl_angular_velocity.err.y + ctrl_angular_velocity.err_d.y + ctrl_angular_velocity.err_i.y ) \
																		 );
    ctrl_angular_velocity.out.z = 3 *( ctrl_angular_velocity.FB *LIMIT((0.45f + 0.55f*ctrl_attitude.err_weight.z),0,1)*except_AS.z + \
																		   ( 1 - ctrl_angular_velocity.FB ) *ctrl_angular_velocity.PID[PIDYAW].kp   *( ctrl_angular_velocity.err.z + ctrl_angular_velocity.err_d.z + ctrl_angular_velocity.err_i.z ) \
																		 );
    /* 电机油门量控制 */
    Thr_Ctrl(T);
    /* 角速度控制量 转换到 电机转速的输出量 */
    All_Out(ctrl_angular_velocity.out.x,ctrl_angular_velocity.out.y,ctrl_angular_velocity.out.z);
    /* 记录角速度误差积分 */
    ctrl_angular_velocity.err_old.x = ctrl_angular_velocity.err.x;
    ctrl_angular_velocity.err_old.y = ctrl_angular_velocity.err.y;
    ctrl_angular_velocity.err_old.z = ctrl_angular_velocity.err.z;
    /* 记录角速度数据 */
    g_old[A_X] =  mpu6050.Gyro_deg.x ;
    g_old[A_Y] = -mpu6050.Gyro_deg.y ;
    g_old[A_Z] = -mpu6050.Gyro_deg.z ;
}

