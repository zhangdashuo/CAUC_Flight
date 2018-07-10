/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file imu.c
*@version V1.0
*@date  2018/5/24
*@brief 姿态解算文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#include "stm32f4xx.h"
#include "imu.h"
#include "driver_ak8975.h"
#include "driver_mpu6050.h"
#include "mymath.h"
#include "filter.h"

#define Kp 0.3f // proportional gain governs rate of convergence to accelerometer/magnetometer（light原值为0.6）
#define Ki 0.0f // 0.001  integral gain governs rate of convergence of gyroscope biases（light原值为0.1）

/* 角度转弧度比例 */
#define ANGLE_TO_RADIAN 0.017453293f
/* 2倍角度转弧度比例 */
#define IMU_INTEGRAL_LIM  0.034906585f
/* 加速计均方根积分滤波常量 */
#define NORM_ACC_LPF_HZ 10
/* 误差积分滤波常量 */
#define REF_ERR_LPF_HZ  1

/* 加速度：由下向上方向的加速度在加速度计的分量 */
xyz_f_t reference_v;
/* 加速度：由南向北方向的加速度在加速度计的分量 *//* 加速度：由东向西方向的加速度在加速度计的分量 */
xyz_f_t north,west;
/* 数据处理过程量结构体 */
ref_t 	ref;

/* 最终计算出的姿态 单位 角度 */
float IMU_Roll,IMU_Pitch,IMU_Yaw;

/* 四元数的W X Y Z */
float ref_q[4] = {1,0,0,0};

/* 加速计均方根 *//* 四元数均方根 */
float norm_acc,norm_q;
/* 加速计均方根积分滤波 */
float norm_acc_lpf;

/* 磁力计均方根 */ /* 磁力计数据 除以 均方根 */
float mag_norm ,mag_norm_xyz ;

/* 匹配好方位的磁力计数据 */
xyz_f_t mag_sim_3d;

/* 解锁判断标志
0未解锁 1已经解锁 */
extern u8 unlocked_to_fly;

/*----------------------------------------------------------
 + 实现功能：引用加速度计分量作为参考，磁力方位匹配计算
----------------------------------------------------------*/
void mag_3d_trans(xyz_f_t *ref, xyz_f_t *in, xyz_f_t *out)
{
    static s8 pn;
    static float h_tmp_x,h_tmp_y;

    arm_sqrt_f32( (ref->z)*(ref->z) + (ref->y)*(ref->y) , &h_tmp_x);
    arm_sqrt_f32( (ref->z)*(ref->z) + (ref->x)*(ref->x) , &h_tmp_y);

    pn = ref->z < 0? -1 : 1;

    out->x = ( h_tmp_x *in->x - pn *ref->x *in->z ) ;
    out->y = ( pn *h_tmp_y *in->y - ref->y *in->z ) ;
    out->z = ref->x *in->x + ref->y *in->y + ref->z *in->z ;
}

/*----------------------------------------------------------
 + 实现功能：由任务调度调用周期5ms
 + 调用参数：两次调用时间差的一半
----------------------------------------------------------*/
void Call_IMUupdate(float half_T)
{
    /* 陀螺仪数据赋值 */
    float gx = mpu6050.Gyro_deg.x;
    float gy = mpu6050.Gyro_deg.y;
    float gz = mpu6050.Gyro_deg.z;
    /* 加速度计数据赋值 */
    float ax = mpu6050.Acc.x;
    float ay = mpu6050.Acc.y;
    float az = mpu6050.Acc.z;
    /* 误差积分滤波比例 */
    float ref_err_lpf_hz;
    static float yaw_correct;
    static float yaw_mag;

//    static char temp_i;
    static xyz_f_t mag_tmp;
    float mag_norm_tmp;

    /* 积分滤波比例 */
    mag_norm_tmp = 20 *(6.28f *half_T);

    /* 磁力计均方根计算 */
    arm_sqrt_f32 ( ak8975.Mag_Val.x * ak8975.Mag_Val.x + ak8975.Mag_Val.y * ak8975.Mag_Val.y + ak8975.Mag_Val.z * ak8975.Mag_Val.z  ,  &mag_norm_xyz );

    /* 数据正常 */
    if( mag_norm_xyz != 0)
    {
        /* 磁力计分量 均方根计算 */
        mag_tmp.x += mag_norm_tmp *( (float)ak8975.Mag_Val.x /( mag_norm_xyz ) - mag_tmp.x);
        mag_tmp.y += mag_norm_tmp *( (float)ak8975.Mag_Val.y /( mag_norm_xyz ) - mag_tmp.y);
        mag_tmp.z += mag_norm_tmp *( (float)ak8975.Mag_Val.z /( mag_norm_xyz ) - mag_tmp.z);
    }

    /* 引用加速度计分量作为参考，磁力方位匹配计算 */
    mag_3d_trans(&reference_v,&mag_tmp,&mag_sim_3d);

    /* 匹配好方位的磁力计数据 均方根计算 */
    arm_sqrt_f32(mag_sim_3d.x * mag_sim_3d.x + mag_sim_3d.y *mag_sim_3d.y , &mag_norm);
    /* 数据正常 */
    if( mag_sim_3d.x != 0 && mag_sim_3d.y != 0 && mag_sim_3d.z != 0 && mag_norm != 0)
        /* 计算Y航向角度 单位角度 */
        yaw_mag = fast_atan2( ( mag_sim_3d.x/mag_norm ) , ( mag_sim_3d.y/mag_norm) ) *57.324841f;

    /* 加速度：由下向上方向的加速度在加速度计X分量 */
    reference_v.x = 2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2]);
    /* 加速度：由下向上方向的加速度在加速度计Y分量 */
    reference_v.y = 2*(ref_q[2]*ref_q[3] + ref_q[0]*ref_q[1]);
    /* 加速度：由下向上方向的加速度在加速度计Z分量 */
    reference_v.z = 1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2]);
    /* 加速度：由南向北方向的加速度在加速度计X分量 */
    north.x = 1 - 2*(ref_q[3]*ref_q[3] + ref_q[2]*ref_q[2]);
    /* 加速度：由南向北方向的加速度在加速度计Y分量 */
    north.y = 2* (-ref_q[0]*ref_q[3] + ref_q[1]*ref_q[2]);
    /* 加速度：由南向北方向的加速度在加速度计Z分量 */
    north.z = 2* (+ref_q[0]*ref_q[2]  - ref_q[1]*ref_q[3]);
    /* 加速度：由东向西方向的加速度在加速度计X分量 */
    west.x = 2* (+ref_q[0]*ref_q[3] + ref_q[1]*ref_q[2]);
    /* 加速度：由东向西方向的加速度在加速度计Y分量 */
    west.y = 1 - 2*(ref_q[3]*ref_q[3] + ref_q[1]*ref_q[1]);
    /* 加速度：由东向西方向的加速度在加速度计Z分量 */
    west.z = 2* (-ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]);

    /* 加速计均方根 估值约4096 */
    arm_sqrt_f32( ax*ax + ay*ay + az*az , &norm_acc );
    /* 加速计均方根积分滤波 估值约4096 */
    norm_acc_lpf +=  NORM_ACC_LPF_HZ *(6.28f *half_T) *(norm_acc - norm_acc_lpf);

    /* 判断加速度计数据范围 */
    if(ABS(ax)<4400 && ABS(ay)<4400 && ABS(az)<4400 )
    {

        /* 加速度计数值 除以加速计均方根 */
        ax = ax / norm_acc;
        ay = ay / norm_acc;
        az = az / norm_acc;
        /* 数据正常 */
        if( 3800 < norm_acc && norm_acc < 4400 )
        {
            /* 叉乘得到误差 */
            ref.err_tmp.x = ay*reference_v.z - az*reference_v.y;
            ref.err_tmp.y = az*reference_v.x - ax*reference_v.z;
            /* 误差积分滤波比例 */
            ref_err_lpf_hz = REF_ERR_LPF_HZ *(6.28f *half_T);
            /* 误差积分滤波 */
            ref.err_lpf.x += ref_err_lpf_hz *( ref.err_tmp.x  - ref.err_lpf.x );
            ref.err_lpf.y += ref_err_lpf_hz *( ref.err_tmp.y  - ref.err_lpf.y );
            ref.err.x = ref.err_lpf.x;
            ref.err.y = ref.err_lpf.y;
        }
    }
    /* 数据异常 */
    else
    {
        ref.err.x = 0 ;
        ref.err.y = 0 ;
    }
    /* 误差积分 */
    ref.err_Int.x += ref.err.x *Ki *2 *half_T ;
    ref.err_Int.y += ref.err.y *Ki *2 *half_T ;
    ref.err_Int.z += ref.err.z *Ki *2 *half_T ;
    /* 积分限幅 */
    ref.err_Int.x = LIMIT(ref.err_Int.x, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
    ref.err_Int.y = LIMIT(ref.err_Int.y, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
    ref.err_Int.z = LIMIT(ref.err_Int.z, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
    /* 方向朝上 */
    if( reference_v.z >= 0.0f )
    {
        /* 已经解锁，防止磁力计受干扰只可低速融合 */
        if( unlocked_to_fly )
        {
            /* 磁力计计算姿态按较小权重算融合 */
            yaw_correct = Kp *0.1f *To_180_degrees(yaw_mag - IMU_Yaw);
        }
        /* 没有解锁，视作开机时刻，快速融合 */
        else
        {
            /* 磁力计计算姿态按较大权重算融合 */
            yaw_correct = Kp *0.2f *To_180_degrees(yaw_mag - IMU_Yaw);
        }
    }
    /* 方向朝下 */
    else
    {
        yaw_mag+=180.0f;
        To_180_degrees(yaw_mag);

        /* 已经解锁，防止磁力计受干扰只可低速融合 */
        if( unlocked_to_fly )
        {
            /* 磁力计计算姿态按较小权重算融合 */
            yaw_correct = Kp *0.1f *To_180_degrees(yaw_mag - IMU_Yaw);
        }
        /* 没有解锁，视作开机时刻，快速融合 */
        else
        {
            /* 磁力计计算姿态按较大权重算融合 */
            yaw_correct = Kp *0.2f *To_180_degrees(yaw_mag - IMU_Yaw);
        }
    }

    /* 引用陀螺仪数据  */
    ref.g.x = (gx - reference_v.x *yaw_correct) *ANGLE_TO_RADIAN + ( Kp*(ref.err.x + ref.err_Int.x) ) ;
    ref.g.y = (gy - reference_v.y *yaw_correct) *ANGLE_TO_RADIAN + ( Kp*(ref.err.y + ref.err_Int.y) ) ;
    ref.g.z = (gz - reference_v.z *yaw_correct) *ANGLE_TO_RADIAN;

    /* 更新四元数 */
    ref_q[0] = ref_q[0] +(-ref_q[1]*ref.g.x - ref_q[2]*ref.g.y - ref_q[3]*ref.g.z)*half_T;
    ref_q[1] = ref_q[1] + (ref_q[0]*ref.g.x + ref_q[2]*ref.g.z - ref_q[3]*ref.g.y)*half_T;
    ref_q[2] = ref_q[2] + (ref_q[0]*ref.g.y - ref_q[1]*ref.g.z + ref_q[3]*ref.g.x)*half_T;
    ref_q[3] = ref_q[3] + (ref_q[0]*ref.g.z + ref_q[1]*ref.g.y - ref_q[2]*ref.g.x)*half_T;

    /* 计算四元数均方根 */
    arm_sqrt_f32( ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3] , &norm_q );

    /* 四元数 除以四元数均方根，保证范围1以内 */
    ref_q[0] = ref_q[0] / norm_q;
    ref_q[1] = ref_q[1] / norm_q;
    ref_q[2] = ref_q[2] / norm_q;
    ref_q[3] = ref_q[3] / norm_q;

    /* 四元数转换到欧拉角 算法 */
    IMU_Roll = fast_atan2(2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]),1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2])) *57.324841f ;
    IMU_Pitch = asin(2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2])) *57.324841f ;
    IMU_Yaw = fast_atan2(2*(-ref_q[1]*ref_q[2] - ref_q[0]*ref_q[3]), 2*(ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1]) - 1) *57.324841f ;
}
