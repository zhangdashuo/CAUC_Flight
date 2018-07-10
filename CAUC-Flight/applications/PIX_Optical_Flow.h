/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file main.c
*@version V1.0
*@date  2018/5/24
*@brief 光流驱动头文件
*@design 无人机研究小组
	     谭政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#ifndef _OPTICAL_FLOW_H
#define	_OPTICAL_FLOW_H

#include "stm32f4xx.h"

/*下面开始加入PIX光流读取代码 add by zhangshuo*/
typedef struct
{
	float average;//Flow in m in x-sensor direction, angular-speed compensated
	float originf;
	int16_t origin;
}FLOW_DATA;

typedef struct
{
	uint64_t  time_sec;
	u8   id;
	FLOW_DATA flow_x;
	FLOW_DATA flow_y;
	FLOW_DATA flow_comp_x;//Flow in m in x-sensor direction, angular-speed compensated
	FLOW_DATA flow_comp_y;
	u8 quality; //Optical flow quality / confidence. 0: bad, 255: maximum quality
	FLOW_DATA hight;//ground_distance        float        Ground distance in m. Positive value: distance known. Negative value: Unknown distance               
}FLOW;

typedef struct
{
	uint64_t time_usec; ///< Timestamp (microseconds, synced to UNIX time or since system boot)
	uint32_t integration_time_us; ///< Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
	float integrated_x; ///< Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
	float integrated_y; ///< Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
	float integrated_xgyro; ///< RH rotation around X axis (rad)
	float integrated_ygyro; ///< RH rotation around Y axis (rad)
	float integrated_zgyro; ///< RH rotation around Z axis (rad)
	uint32_t time_delta_distance_us; ///< Time in microseconds since the distance was sampled.
	float distance; ///< Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative value: Unknown distance.
	int16_t temperature; ///< Temperature * 100 in centi-degrees Celsius
	uint8_t sensor_id; ///< Sensor ID
	uint8_t quality; ///< Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
}FLOW_RAD;

typedef struct
{
	FLOW_DATA flow_x;
	FLOW_DATA flow_y;
	FLOW_DATA flow_comp_x;//Flow in m in x-sensor direction, angular-speed compensated
	FLOW_DATA flow_comp_y;
	float scale_rad_fix;
	float scale_rad_fix_comp;
	FLOW_DATA hight;//ground_distance        float        Ground distance in m. Positive value: distance known. Negative value: Unknown distance               
}FLOW_FIX;






/* 读取X偏移量原始数据  */
extern int Optical_flow_org_x;
/* 读取Y偏移量原始数据 */
extern int Optical_flow_org_y;
/* 读取Confidence原始数据 */
extern int Optical_flow_confidence;

/* 换算到X像素速度  光流像素要结合高度才能算出速度 值范围约+-200.000 */
extern float Optical_flow_x;
/* 换算到Y像素速度  光流像素要结合高度才能算出速度 值范围约+-200.000 */
extern float Optical_flow_y;
/* 读取画面不变率 值范围0-100 */
extern float Optical_flow_con;

/*----------------------------------------------------------
 + 实现功能：数据接收并保存
 + 调用参数：接收到的单字节数据
----------------------------------------------------------*/
extern void Optical_Flow_Receive_Prepare(u8 data);

/*----------------------------------------------------------
 + 实现功能：光流初始化
----------------------------------------------------------*/
extern void Optical_Flow_init(void);

#endif

