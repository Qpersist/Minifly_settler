/******************** (C) COPYRIGHT 2014 ANO Tech ***************************
 * 作者		 ：匿名科创
 * 文件名  ：ANO_RC.c
 * 描述    ：参数读取和保存
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/

#include "ANO_RC.h"
#include "ANO_Data_Transfer.h"
#include "ANO_Data.h"
#include "Receive.h"
#define UN_YAW_VALUE  300
#define UN_THR_VALUE -400

u8 fly_ready = 0;
u16 fly_ready_cnt = 0;

u16 RX_CH[CH_NUM];
s16 CH_N[CH_NUM];

void unlock()
{
	//解锁检测
	if(CH_N[THR] < UN_THR_VALUE)
	{
		if(CH_N[YAW]>UN_YAW_VALUE)
		{
			if(fly_ready_cnt<10000)
			{
				fly_ready_cnt++;
			}
			if(fly_ready_cnt>200) //200*dT秒
			{
				fly_ready = 1;
			}
		}
		else
		{
			if(CH_N[YAW]>-UN_YAW_VALUE)
			{
				fly_ready_cnt = 0;
			}
		}
	}
	else
	{
		fly_ready_cnt = 0;
	}
	
	//上锁检测
	if(CH_N[THR] < UN_THR_VALUE)
	{
		if(CH_N[YAW]<-UN_YAW_VALUE)
		{
			if(fly_ready_cnt<10000)
			{
				fly_ready_cnt++;
			}
			if(fly_ready_cnt>200) //200*dT秒
			{
				fly_ready = 0;
			}
		}
		else
		{
			if(CH_N[YAW]<UN_YAW_VALUE)
			{
				fly_ready_cnt = 0;
			}
		}
	}
	else
	{
		fly_ready_cnt = 0;
	}
	
	if(CH_N[THR]>-350)
	{
		flag.thr_low = 0;//油门非低
	}
	else
	{
		flag.thr_low = 1;//油门拉低
	}
		
}

void RC_duty(float dT) //建议2ms调用一次
{
//	u8 i ;
//	for(i=0;i<CH_NUM;i++)
//	{
		CH_N[0] = g_t_Rc_D_CMS.ROLL- 1500;
		CH_N[1] = g_t_Rc_D_CMS.PITCH- 1500;
		CH_N[2] = g_t_Rc_D_CMS.THROTTLE - 1500;
		CH_N[3] = g_t_Rc_D_CMS.YAW - 1500;
//	}
//解锁监测	
	unlock();
//摇杆触发功能监测
//	stick_function(dT);	
//失控保护检查
	fail_safe_check();
}

void fail_safe()
{
	fly_ready = 0;
	CH_N[THR] = -500;
	CH_N[ROL] = 0;
	CH_N[PIT] = 0;
	CH_N[YAW] = 0;
}


void fail_safe_check() //dT秒调用一次
{
	static u16 cnt;
	
	if(flag.NS == 0)
	{
		cnt++;
	}
	else
	{
		cnt = 0;
	}

	if(cnt >= 500) //500*dT 秒
	{
		flag.signal_loss = 1;
		fail_safe();
	}
	else
	{
		flag.signal_loss = 0;
	}
	
	flag.NS = 0;
}

void stick_function_check(float dT,_stick_f_c_st *sv,u8 times_n,u16 reset_time_ms,u8 en,u8 *trig)
{
	if(en)
	{
		sv->s_cnt = 0; //清除计时
		if(sv->s_state==0)
		{
			sv->s_now_times++;
			sv->s_state = 1;
		}
	}
	else
	{
		sv->s_state = 0;
		/////
		sv->s_cnt += 1000*dT;
		if(sv->s_cnt>reset_time_ms)
		{
			sv->s_now_times = 0; //清除记录次数
		}
	}

	if(sv->s_now_times>=times_n)
	{
		*trig = 1;            //触发功能标记
		sv->s_now_times = 0;
	}

}

_stick_f_c_st cali_gyro,cali_acc;
u8 stick_fun_up,stick_fun_dw;
void stick_function(float dT)
{
	//////////////状态监测
	if(fly_ready==0)
	{
		if(flag.thr_low)
		{
			if(CH_N[PIT]>350)
			{
				stick_fun_up = 1;
			}
			else if(CH_N[PIT]<300)
			{
				stick_fun_up = 0;
			}
			
			if(CH_N[PIT]<-350)
			{
				stick_fun_dw = 1;
			}
			else if(CH_N[PIT]>-300)
			{
				stick_fun_dw = 0;
			}
		}
	}
	///////////////
	stick_function_check(dT,&cali_gyro,2,1000,stick_fun_up,&sensor.gyr_CALIBRATE);
	
	stick_function_check(dT,&cali_acc,3,1000,stick_fun_up,&sensor.acc_CALIBRATE);
	
}





/******************* (C) COPYRIGHT 2016 ANO TECH *****END OF FILE************/

