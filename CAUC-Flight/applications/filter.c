/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file filter.c
*@version V1.0
*@date  2018/5/24
*@brief 数据滤波文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#include "stm32f4xx.h"
#include "filter.h"
#include "mymath.h"

/* 计算求取中位数组总数 */
#define MED_WIDTH_NUM 11
/* 计算求取浮点中位数组项 */
#define MED_FIL_ITEM  2
/* 计算求取整形中位数组项 */
#define MED_FIL_ITEM_int  2

/* 计算求取浮点中位数组 */
float med_filter_tmp[MED_FIL_ITEM][MED_WIDTH_NUM ];
/* 计算求取整形中位数组 */
int med_filter_tmp_int[MED_FIL_ITEM_int][MED_WIDTH_NUM ];
/* 计算求取整形中位数组下标 */
u8 med_fil_cnt[MED_FIL_ITEM];

/*----------------------------------------------------------
 + 实现功能：float类型数据滑动窗口滤波
 + 调用参数：in：加入的数据 moavarray[]：滑动窗口数组 len：求取数据个数 fil_cnt[2]：数组下标
 + 调用参数：*out ：算出的中位数
----------------------------------------------------------*/
void Moving_Average(float in,float moavarray[],u16 len ,u16 fil_cnt[2],float *out)
{
    u16 width_num = len ;
    if( ++fil_cnt[0] > width_num )
    {
        fil_cnt[0] = 0; //now
        fil_cnt[1] = 1; //old
    }
    else
        fil_cnt[1] = (fil_cnt[0] == width_num)? 0 : (fil_cnt[0] + 1);
    moavarray[ fil_cnt[0] ] = in;
    *out += ( in - ( moavarray[ fil_cnt[1] ]  ) )/(float)( width_num ) ;
}

/*----------------------------------------------------------
 + 实现功能：float类型数据求中位数
 + 调用参数：item：项目 width_num：求取数据个数 in：加入的数据
 + 返回参数：算出的中位数
----------------------------------------------------------*/
float Moving_Median(u8 item,u8 width_num,float in)
{
    u8 i,j;
    float t;
    float tmp[MED_WIDTH_NUM];

    if(item >= MED_FIL_ITEM || width_num >= MED_WIDTH_NUM )
        return 0;
    else
    {
        if( ++med_fil_cnt[item] >= width_num )
            med_fil_cnt[item] = 0;
        med_filter_tmp[item][ med_fil_cnt[item] ] = in;
        for(i=0; i<width_num; i++)
            tmp[i] = med_filter_tmp[item][i];
        for(i=0; i<width_num-1; i++)
        {
            for(j=0; j<(width_num-1-i); j++)
            {
                if(tmp[j] > tmp[j+1])
                {
                    t = tmp[j];
                    tmp[j] = tmp[j+1];
                    tmp[j+1] = t;
                }
            }
        }
        return ( tmp[(u16)width_num/2] );
    }
}

/*----------------------------------------------------------
 + 实现功能：int类型数据求中位数
 + 调用参数：item：项目 width_num：求取数据个数 in：加入的数据
 + 返回参数：算出的中位数
----------------------------------------------------------*/
int Moving_Median_int(u8 item,u8 width_num,int in_int)
{
    u8 i,j;
    int t_int;
    int tmp[MED_WIDTH_NUM];

    if(item >= MED_FIL_ITEM || width_num >= MED_WIDTH_NUM )
        return 0;
    else
    {
        if( ++med_fil_cnt[item] >= width_num )
            med_fil_cnt[item] = 0;
        med_filter_tmp_int[item][ med_fil_cnt[item] ] = in_int;
        for(i=0; i<width_num; i++)
            tmp[i] = med_filter_tmp_int[item][i];
        for(i=0; i<width_num-1; i++)
        {
            for(j=0; j<(width_num-1-i); j++)
            {
                if(tmp[j] > tmp[j+1])
                {
                    t_int = tmp[j];
                    tmp[j] = tmp[j+1];
                    tmp[j+1] = t_int;
                }
            }
        }
        return ( tmp[(u16)width_num/2] );
    }
}

