/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file driver_parameter.c
*@version V1.0
*@date  2018/5/24
*@brief 默认参数及校准数据文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#include "stm32f4xx.h"
#include "driver_parameter.h"
#include "driver_mpu6050.h"
#include "driver_ak8975.h"
#include "ctrl.h"
#include "string.h"
#include "height_ctrl.h"
#include "device_fatfs.h"
#include "database.h"

pid_setup_t pid_setup;

/* 从文件读取所有配置参数 */
static int32_t Para_ReadSettingFromFile(void);
/* 保存所有配置参数到文件 */
static int32_t Para_WriteSettingToFile(void);
/* 加载校准、主要PID参数 */
static void  Param_SetSettingToFC(void);

/*----------------------------------------------------------
 + 实现功能：PID参数恢复默认
----------------------------------------------------------*/
void Para_ResetToFactorySetup(void)
{
    /* 如果挂载不成功，进行格式化 */
    f_mkfs("0:",1,0);

    /* 以下是所有 PID 默认值，只有收到串口的 恢复默认参数 命令，才会恢复到这些参数 */

    /* ROLL速率 */
    pid_setup.groups.ctrl1.roll.kp  = 0.800;
    pid_setup.groups.ctrl1.roll.ki  = 0.100;
    pid_setup.groups.ctrl1.roll.kd  = 2.000;
    pid_setup.groups.ctrl1.roll.kdamp  = 1;
    /* PITCH速率 */
    pid_setup.groups.ctrl1.pitch.kp = 0.800;
    pid_setup.groups.ctrl1.pitch.ki = 0.100;
    pid_setup.groups.ctrl1.pitch.kd = 2.000;
    pid_setup.groups.ctrl1.pitch.kdamp = 1;
    /* YAW速率 */
    pid_setup.groups.ctrl1.yaw.kp   = 1.200;
    pid_setup.groups.ctrl1.yaw.ki   = 1.000;
    pid_setup.groups.ctrl1.yaw.kd   = 1.000;
    pid_setup.groups.ctrl1.yaw.kdamp   = 1;
    /* ROLL自稳 */
    pid_setup.groups.ctrl2.roll.kp  = 0.500;
    pid_setup.groups.ctrl2.roll.ki  = 0.050;
    pid_setup.groups.ctrl2.roll.kd  = 0.300;
    /* PITCH自稳 */
    pid_setup.groups.ctrl2.pitch.kp = 0.500;
    pid_setup.groups.ctrl2.pitch.ki = 0.050;
    pid_setup.groups.ctrl2.pitch.kd = 0.300;
    /* YAW自稳 */
    pid_setup.groups.ctrl2.yaw.kp   = 0.200;
    pid_setup.groups.ctrl2.yaw.ki   = 0.050;
    pid_setup.groups.ctrl2.yaw.kd   = 0.100;
    /* 气压计高度 */
    pid_setup.groups.hc_sp.kp = 0.3f;
    pid_setup.groups.hc_sp.ki = 0.12f;
    pid_setup.groups.hc_sp.kd = 1.4f;
    /* 超声波高度 */
    pid_setup.groups.hc_height.kp = 1.5f;
    pid_setup.groups.hc_height.ki = 0.0f;
    pid_setup.groups.hc_height.kd = 2.5f;
    /* 位置速率 */
    pid_setup.groups.ctrl3.kp = 1.5f;
    pid_setup.groups.ctrl3.ki = 1.0f;
    pid_setup.groups.ctrl3.kd = 1.0f;
    /* 位置保持 */
    pid_setup.groups.ctrl4.kp = 1.5f;
    pid_setup.groups.ctrl4.ki = 1.0f;
    pid_setup.groups.ctrl4.kd = 1.0f;
	
	/* 光流速度保持 */
    pid_setup.groups.ctrl6.kp = 0.1f;
    pid_setup.groups.ctrl6.ki = 0.1f;
    pid_setup.groups.ctrl6.kd = 0.1f;

    /* 保存所有配置参数到文件 */
    Para_WriteSettingToFile();
    /* 加载校准、主要PID参数 */
    Param_SetSettingToFC();
    /* 次要PID的配置参数初始化 */
    PID_Para_Init();
}

/*----------------------------------------------------------
 + 实现功能：加载校准、主要PID参数
----------------------------------------------------------*/
/* 保存传感器的校准参数结构体 */
static sensor_setup_t sensor_setup;
static void  Param_SetSettingToFC(void)
{
    /* 拷贝并覆盖数据 */
    memcpy(&mpu6050.Acc_Offset,&sensor_setup.Offset.Accel,sizeof(xyz_f_t));
    memcpy(&mpu6050.Gyro_Offset,&sensor_setup.Offset.Gyro,sizeof(xyz_f_t));
    memcpy(&ak8975.Mag_Offset,&sensor_setup.Offset.Mag,sizeof(xyz_f_t));
    memcpy(&mpu6050.vec_3d_cali,&sensor_setup.Offset.vec_3d_cali,sizeof(xyz_f_t));
    mpu6050.Acc_Temprea_Offset = sensor_setup.Offset.Acc_Temperature;
    mpu6050.Gyro_Temprea_Offset = sensor_setup.Offset.Gyro_Temperature;
    memcpy(&ctrl_angular_velocity.PID[PIDROLL],&pid_setup.groups.ctrl1.roll,sizeof(pid_t));
    memcpy(&ctrl_angular_velocity.PID[PIDPITCH],&pid_setup.groups.ctrl1.pitch,sizeof(pid_t));
    memcpy(&ctrl_angular_velocity.PID[PIDYAW],&pid_setup.groups.ctrl1.yaw,sizeof(pid_t));
    memcpy(&ctrl_attitude.PID[PIDROLL],&pid_setup.groups.ctrl2.roll,sizeof(pid_t));
    memcpy(&ctrl_attitude.PID[PIDPITCH],&pid_setup.groups.ctrl2.pitch,sizeof(pid_t));
    memcpy(&ctrl_attitude.PID[PIDYAW],&pid_setup.groups.ctrl2.yaw,sizeof(pid_t));
}

/*----------------------------------------------------------
 + 实现功能：次要PID的配置参数初始化
----------------------------------------------------------*/
void PID_Para_Init()
{
    /* 控制参数初始化 */
    Ctrl_Para_Init();
    /* 气压定高参数初始化 */
    WZ_Speed_PID_Init();
    /* 超声波定高参数初始化 */
    Ultra_PID_Init();
}

/*----------------------------------------------------------
 + 实现功能：所有参数初始化
----------------------------------------------------------*/
void Para_Init()
{
    /* 从文件读取所有配置参数 */
    int32_t result = Para_ReadSettingFromFile();
    /* 异常判断 */
    if(result < 0)
        /* PID所有参数恢复默认 */
        Para_ResetToFactorySetup();
    /* 加载校准、主要PID参数 */
    Param_SetSettingToFC();
    /* 次要PID的配置参数初始化 */
    PID_Para_Init();
}

/*----------------------------------------------------------
 + 实现功能：保存主要PID的配置参数并保存到文件
----------------------------------------------------------*/
void Param_SavePID(void)
{
    /* 拷贝并覆盖数据 */
    memcpy(&pid_setup.groups.ctrl1.roll,&ctrl_angular_velocity.PID[PIDROLL],sizeof(pid_t));
    memcpy(&pid_setup.groups.ctrl1.pitch,&ctrl_angular_velocity.PID[PIDPITCH],sizeof(pid_t));
    memcpy(&pid_setup.groups.ctrl1.yaw,&ctrl_angular_velocity.PID[PIDYAW],sizeof(pid_t));
    memcpy(&pid_setup.groups.ctrl2.roll,&ctrl_attitude.PID[PIDROLL],sizeof(pid_t));
    memcpy(&pid_setup.groups.ctrl2.pitch,&ctrl_attitude.PID[PIDPITCH],sizeof(pid_t));
    memcpy(&pid_setup.groups.ctrl2.yaw,&ctrl_attitude.PID[PIDYAW],sizeof(pid_t));
    /* 保存所有配置参数到文件 */
    Para_WriteSettingToFile();
}

/*----------------------------------------------------------
 + 实现功能：由任务调度调用周期20ms
----------------------------------------------------------*/
extern u16 flash_save_en_cnt;
void Parameter_Save()
{
    if( flash_save_en_cnt !=0 )
        flash_save_en_cnt++;

    /* 调度周期：20 *60 = 1200ms */
    if( flash_save_en_cnt > 60 )
    {
        flash_save_en_cnt = 0;
        if( !unlocked_to_fly )
            Param_SavePID();
    }
}

/*----------------------------------------------------------
 + 实现功能：保存加速度计的校准参数
----------------------------------------------------------*/
void Param_SaveAccelOffset(xyz_f_t *offset)
{
    /* 拷贝并覆盖数据 */
    memcpy(&mpu6050.Acc_Offset,offset,sizeof(xyz_f_t));
    memcpy(&sensor_setup.Offset.Accel, offset,sizeof(xyz_f_t));
    sensor_setup.Offset.Acc_Temperature = mpu6050.Acc_Temprea_Offset ;
    /* 保存所有配置参数到文件 */
    Para_WriteSettingToFile();
}

/*----------------------------------------------------------
 + 实现功能：保存陀螺仪的校准参数
----------------------------------------------------------*/
void Param_SaveGyroOffset(xyz_f_t *offset)
{
    /* 拷贝并覆盖数据 */
    memcpy(&mpu6050.Gyro_Offset,offset,sizeof(xyz_f_t));
    memcpy(&sensor_setup.Offset.Gyro, offset,sizeof(xyz_f_t));
    sensor_setup.Offset.Gyro_Temperature = mpu6050.Gyro_Temprea_Offset ;
    /* 保存所有配置参数到文件 */
    Para_WriteSettingToFile();
}

/*----------------------------------------------------------
 + 实现功能：保存磁力计的校准参数
----------------------------------------------------------*/
void Param_SaveMagOffset(xyz_f_t *offset)
{
    /* 拷贝并覆盖数据 */
    memcpy(&ak8975.Mag_Offset,offset,sizeof(xyz_f_t));
    memcpy(&sensor_setup.Offset.Mag, offset,sizeof(xyz_f_t));
    /* 保存所有配置参数到文件 */
    Para_WriteSettingToFile();
}

/* 文件相关定义 */
static  FATFS fs;
static 	FIL file;
static 	DIR DirInf;

/* Mount/Unmount a logical drive */
extern FRESULT f_mount (FATFS* fs, const TCHAR* path, BYTE opt);
/* Create a file system on the volume */
extern FRESULT f_mkfs (const TCHAR* path, BYTE sfd, UINT au);

/*----------------------------------------------------------
 + 实现功能：从文件读取所有配置参数
----------------------------------------------------------*/
static int32_t Para_ReadSettingFromFile(void)
{
    FRESULT result;
    UINT bw;

    /* 挂载文件系统 */
    result = f_mount(&fs, "0:", 1);
    if (result != FR_OK)
    {
        /* 如果挂载不成功，进行格式化 */
        result = f_mkfs("0:",0,0);
        if (result == FR_OK)
        {
            /* 重新进行挂载 */
            result = f_mount(&fs, "0:", 0);
            if (result != FR_OK)
            {
                /* 卸载文件系统 */
                f_mount(NULL, "0:", 0);
                return -2 ;
            }
        }
        else
            return -1;
    }

    /* 打开根文件夹 */
    result = f_opendir(&DirInf, "/");
    if (result != FR_OK)
    {
        /* 卸载文件系统 */
        f_mount(NULL, "0:", 0);
        return -3;
    }

    /* 打开文件 */
    result = f_open(&file, "sensor.bin", FA_OPEN_EXISTING | FA_READ);
    if (result !=  FR_OK)
    {
        /* 卸载文件系统 */
        f_mount(NULL, "0:", 0);
        /* 文件不存在 */
        return -4;
    }

    /* 读取Sensor配置文件 */
    result = f_read(&file, &sensor_setup.raw_data, sizeof(sensor_setup), &bw);
    if (bw > 0)
    {
        /* 关闭文件*/
        f_close(&file);
        /* 打开文件 */
        result = f_open(&file, "pid.bin", FA_OPEN_EXISTING | FA_READ);
        if (result !=  FR_OK)
        {
            /* 卸载文件系统 */
            f_mount(NULL, "0:", 0);
            return -4;
        }
        /* 读取PID配置文件 */
        result = f_read(&file, &pid_setup.raw_data, sizeof(pid_setup), &bw);
        if(bw > 0)
        {
            /* 关闭文件*/
            f_close(&file);
            /* 卸载文件系统 */
            f_mount(NULL, "0:", 0);
            return 1;
        }
        else
        {
            /* 关闭文件*/
            f_close(&file);
            /* 卸载文件系统 */
            f_mount(NULL, "0:", 0);
            return -4;
        }
    }
    else
    {
        /* 关闭文件*/
        f_close(&file);
        /* 卸载文件系统 */
        f_mount(NULL, "0:", 0);
        return -5;
    }
}

/*----------------------------------------------------------
 + 实现功能：保存所有配置参数到文件
----------------------------------------------------------*/
static int32_t Para_WriteSettingToFile(void)
{
    FRESULT result;
    UINT bw;

    /* 挂载文件系统 */
    result = f_mount(&fs, "0:", 0);
    if (result != FR_OK)
    {
        /* 如果挂载不成功，进行格式化 */
        result = f_mkfs("0:",0,0);
        if (result == FR_OK)
        {
            /* 重新进行挂载 */
            result = f_mount(&fs, "0:", 0);
            if (result != FR_OK)
            {
                /* 卸载文件系统 */
                f_mount(NULL, "0:", 0);
                return -2 ;
            }
        }
        else
            return -1;
    }

    /* 打开根文件夹 */
    result = f_opendir(&DirInf, "/");
    if (result != FR_OK)
    {
        /* 卸载文件系统 */
        f_mount(NULL, "0:", 0);
        return -3;
    }

    /* 打开文件 */
    result = f_open(&file, "sensor.bin", FA_CREATE_ALWAYS | FA_WRITE);
    if (result !=  FR_OK)
    {
        /* 卸载文件系统 */
        f_mount(NULL, "0:", 0);
        return -4;
    }

    /* 写入Sensor配置文件 */
    result = f_write(&file, &sensor_setup.raw_data, sizeof(sensor_setup), &bw);
    if (result == FR_OK)
    {
        /* 关闭文件*/
        f_close(&file);
        /* 打开文件 */
        result = f_open(&file, "pid.bin", FA_CREATE_ALWAYS | FA_WRITE);
        if (result !=  FR_OK)
        {
            /* 卸载文件系统 */
            f_mount(NULL, "0:", 0);
            return -4;
        }
        /* 写入PID配置文件 */
        result = f_write(&file, &pid_setup.raw_data, sizeof(pid_setup), &bw);
        if(result == FR_OK)
        {
            /* 关闭文件*/
            f_close(&file);
            /* 卸载文件系统 */
            f_mount(NULL, "0:", 0);
            return 1;
        }
        else
        {
            /* 关闭文件*/
            f_close(&file);
            /* 卸载文件系统 */
            f_mount(NULL, "0:", 0);
            return -4;
        }
    }
    else
    {
        /* 关闭文件*/
        f_close(&file);
        /* 卸载文件系统 */
        f_mount(NULL, "0:", 0);
        return -5;
    }
}

