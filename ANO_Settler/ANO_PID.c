/******************** (C) COPYRIGHT 2016 ANO Tech ***************************
 * 作者		 ：匿名科创
 * 文件名  ：ANO_PID.c
 * 描述    ：PID函数
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
*****************************************************************************/
#include "ANO_PID.h"


float PID_calculate( float T,            //周期（单位：秒）
										float in_ff,				//前馈值
										float expect,				//期望值（设定值）
										float feedback,			//反馈值（）
										_PID_arg_st *pid_arg, //PID参数结构体
										_PID_val_st *pid_val,	//PID数据结构体
										float inte_lim			//integration limit，积分限幅
										 )	
{
	float out,differential;
	pid_arg->k_inc_d_norm = LIMIT(pid_arg->k_inc_d_norm,0,1);
	
	pid_val->feedback_d = (-1.0f) *(feedback - pid_val->feedback_old) *safe_div(1.0f,T,0);
	
	pid_val->err =  (expect - feedback );
	
	pid_val->err_d = (pid_val->err - pid_val->err_old) *safe_div(1.0f,T,0);
	
	differential = (pid_arg->kd *pid_val->err_d + pid_arg->k_pre_d *pid_val->feedback_d);
	
	if((s16)(100 *pid_arg->inc_hz)!=0)
	{
		LPF_1_(pid_arg->inc_hz,T,differential,pid_val->err_d_lpf );
	}
	else
	{
		pid_val->err_d_lpf = 0;
	}
	
	pid_val->err_i += (pid_val->err + pid_arg->k_pre_d *pid_val->feedback_d )*T;//)*T;//
	pid_val->err_i = LIMIT(pid_val->err_i,-inte_lim,inte_lim);
	
	out = pid_arg->k_ff *in_ff 
	    + pid_arg->kp *pid_val->err  
	    + pid_arg->k_inc_d_norm *pid_val->err_d_lpf + (1.0f-pid_arg->k_inc_d_norm) *differential
    	+ pid_arg->ki *pid_val->err_i;
	
	pid_val->feedback_old = feedback;
	pid_val->err_old = pid_val->err;
	
	return (out);
}
/******************* (C) COPYRIGHT 2016 ANO TECH *****END OF FILE************/


