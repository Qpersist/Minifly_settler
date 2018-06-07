#ifndef __ANO_PID_H
#define __ANO_PID_H

#include "stm32f10x.h"
#include "include.h"
#include "mymath.h"
#include "filter.h"
/*=====================================================================================================================
						 *****
=====================================================================================================================*/
typedef struct
{
	float kp;			 //比例系数
	float ki;			 //积分系数
	float kd;		 	 //微分系数
	float k_pre_d; //previous_d 微分先行
	float inc_hz;  //不完全微分低通系数
	float k_inc_d_norm; //Incomplete 不完全微分 归一（0,1）
	float k_ff;		 //前馈 
	

}_PID_arg_st;


typedef struct
{
	float err;
	float err_old;
	float feedback_old;
	float feedback_d;
	float err_d;
	float err_d_lpf;
	float err_i;
	float ff;
	float pre_d;

}_PID_val_st;

float PID_calculate( float T,            //周期
										float in_ff,				//前馈
										float expect,				//期望值（设定值）
										float feedback,			//反馈值
										_PID_arg_st *pid_arg, //PID参数结构体
										_PID_val_st *pid_val,	//PID数据结构体
										float inte_lim			//integration limit，积分限幅
										   );			//输出
										
#endif

