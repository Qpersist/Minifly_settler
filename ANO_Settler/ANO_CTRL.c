/******************** (C) COPYRIGHT 2016 ANO Tech ***************************
 * 作者		 ：匿名科创
 * 文件名  ：ANO_Scheduler.cpp
 * 描述    ：主函数
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
*****************************************************************************/
#include "ANO_CTRL.h"
#include "ANO_IMU.h"
#include "ANO_Drv_MPU6050.h"
#include "include.h"
#include "ANO_Motor.h"
#include "ANO_RC.h"
#include "ANO_Data.h"

//角度环参数
_PID_arg_st arg_2_rol ;
_PID_arg_st arg_2_pit ;
_PID_arg_st arg_2_yaw ;
//角速度环参数
_PID_arg_st arg_1_rol ;
_PID_arg_st arg_1_pit ;
_PID_arg_st arg_1_yaw ;
//PID_arg_t arg_1_yaw_comp;//compensatory

_PID_arg_st arg_0_hs ;


_PID_val_st val_2_rol;
_PID_val_st val_2_pit;
_PID_val_st val_2_yaw;

_PID_val_st val_1_rol;
_PID_val_st val_1_pit;
_PID_val_st val_1_yaw;
//PID_val_t val_1_yaw_comp;//compensatory

_PID_val_st val_0_hs;

void CTRL_2_PID_Init()
{
	arg_2_rol.kp = 0.8f   *ANO_Param.PID_rol.kp *0.001f;
	arg_2_rol.ki = 1.2f   *ANO_Param.PID_rol.ki *0.001f;
	arg_2_rol.kd = 0.001f *ANO_Param.PID_rol.kd *0.001f;
	arg_2_rol.k_pre_d = 0.0f;
	arg_2_rol.k_ff = 0.0f;
	
	arg_2_pit.kp = 0.8f   *ANO_Param.PID_pit.kp *0.001f;
	arg_2_pit.ki = 1.2f   *ANO_Param.PID_pit.ki *0.001f;
	arg_2_pit.kd = 0.001f *ANO_Param.PID_pit.kd *0.001f;
	arg_2_pit.k_pre_d = 0.0f;
	arg_2_pit.k_ff = 0.0f;

	arg_2_yaw.kp = 0.8f   *ANO_Param.PID_yaw.kp *0.001f;
	arg_2_yaw.ki = 0.01f   *ANO_Param.PID_yaw.ki *0.001f;
	arg_2_yaw.kd = 0.001f   *ANO_Param.PID_yaw.kd *0.001f;
	arg_2_yaw.k_pre_d = 0.0f;
	arg_2_yaw.k_ff = 0.0f;		
}
void CTRL_1_PID_Init()
{
	arg_1_rol.kp = 2.0f   *ANO_Param.PID_rol_s.kp *0.001f;
	arg_1_rol.ki = 2.0f   *ANO_Param.PID_rol_s.ki *0.001f;
	arg_1_rol.kd = 0.0f   ;
	arg_1_rol.k_pre_d = 0.06f *ANO_Param.PID_rol_s.kd *0.001f;
	arg_1_rol.k_ff = 0.01f;
	
	arg_1_pit.kp = 2.0f   *ANO_Param.PID_pit_s.kp *0.001f;
	arg_1_pit.ki = 2.0f   *ANO_Param.PID_pit_s.ki *0.001f;
	arg_1_pit.kd = 0.0f   ;
	arg_1_pit.k_pre_d = 0.06f *ANO_Param.PID_pit_s.kd *0.001f; 
	arg_1_pit.k_ff = 0.001f;

	arg_1_yaw.kp = 4.0f   *ANO_Param.PID_yaw_s.kp *0.001f;
	arg_1_yaw.ki = 2.0f   *ANO_Param.PID_yaw_s.ki *0.001f;
	arg_1_yaw.kd = 0.0f   ;
	arg_1_yaw.k_pre_d = 0.06f *ANO_Param.PID_yaw_s.kd *0.001f;
	arg_1_yaw.k_ff = 0.06f;	
	
}
void CTRL_0_PID_Init()
{
	arg_0_hs.kp = 0.1f *ANO_Param.PID_hs.kp *0.001f;
	arg_0_hs.ki = 0.0f *ANO_Param.PID_hs.ki *0.001f;
	arg_0_hs.kd = 0.0f;
	arg_0_hs.k_pre_d = 0.0f *ANO_Param.PID_hs.kd *0.001f;
	arg_0_hs.k_ff = 1.0f;
	
}

void pid_init(void)
{
	CTRL_2_PID_Init();
	CTRL_1_PID_Init();
	CTRL_0_PID_Init();
}

void CTRL_2(float dT,float weight,_copter_ctrl_st *data) //角度环
{
	(data->out_rol) = PID_calculate(  dT,            			//周期
																	0,						//前馈
																	data->exp_rol,			//期望值（设定值）
																	data->fb_rol,			//反馈值
																	&arg_2_rol, 			//PID参数结构体
																	&val_2_rol,				//PID数据结构体
																	CTRL_2_INTER_LIMIT *weight			//integral limit，积分限幅
																	 );	//输出
	
	(data->out_pit) = PID_calculate(  dT,            			//周期
																	0,						//前馈
																	data->exp_pit,			//期望值（设定值）
																	data->fb_pit,			//反馈值
																	&arg_2_pit, 			//PID参数结构体
																	&val_2_pit,				//PID数据结构体
																	CTRL_2_INTER_LIMIT *weight		//integral limit，积分限幅
																		);	//输出
	
 	(data->out_yaw) = PID_calculate(  dT,            			//周期
																	0,						//前馈
																	data->exp_yaw,			//期望值（设定值）
																	data->fb_yaw,			//反馈值
																	&arg_2_yaw, 			//PID参数结构体
																	&val_2_yaw,				//PID数据结构体
																	CTRL_2_INTER_LIMIT *weight					//integral limit，积分限幅
																		);	//输出

}


void CTRL_1(float dT,float weight,_copter_ctrl_st *data) //角速率环
{
	
	(data->out_rol)  = PID_calculate(  dT,            			//周期
																		data->exp_rol,			//前馈
																		data->exp_rol,			//期望值（设定值）
																		data->fb_rol,			//反馈值
																		&arg_1_rol, 			//PID参数结构体
																		&val_1_rol,				//PID数据结构体
																		CTRL_1_INTER_LIMIT *weight			//integral limit，积分限幅
																		 );			//输出
	
	(data->out_pit) = PID_calculate(  dT,            			//周期
					data->exp_pit,			//前馈
					data->exp_pit,			//期望值（设定值）
					data->fb_pit,			//反馈值
					&arg_1_pit, 			//PID参数结构体
					&val_1_pit,				//PID数据结构体
					CTRL_1_INTER_LIMIT *weight		//integral limit，积分限幅
					  );			//输出
	
	(data->out_yaw) = PID_calculate(  dT,            			//周期
					data->exp_yaw,			//前馈
					data->exp_yaw,			//期望值（设定值）
					data->fb_yaw,			//反馈值
					&arg_1_yaw, 			//PID参数结构体
					&val_1_yaw,				//PID数据结构体
					2 *CTRL_1_INTER_LIMIT *weight		//integral limit，积分限幅
					);			//输出
						
}



void CTRL_0(float dT,float weight,float thr,float exp,float fb,float *out)
{
	*out = PID_calculate(	dT,            		//周期
											thr,				//前馈
											exp,				//期望值（设定值）
											fb,					//反馈值
											&arg_0_hs, 			//PID参数结构体
											&val_0_hs,			//PID数据结构体
											0					//integration limit，积分限幅
												);				//输出&(*out)
	
	//*out = *out *weight;

}
/*=====================================================================================================================
						headless
=====================================================================================================================*/	
void headless_mode(u8 en,float in_x,float in_y,float *out_x,float *out_y)
{
	//开发中
	
	*out_x = in_x;
	*out_y = in_y;
}


/*=====================================================================================================================
						CH_N  1横滚，2俯仰，3油门，4航向 范围：+-500
=====================================================================================================================*/
_copter_ctrl_st ctrl_1; 
_copter_ctrl_st ctrl_2; 

volatile u32 ctrl_2_time; 
volatile u32 ctrl_1_time; 
volatile u32 ctrl_0_time; 

float thr,thr_value,exp_hspeed,exp_hspeed_lpf; 
float out_rol_curve,out_pit_curve,out_yaw_curve;
float est_rol,est_pit,est_yaw;

void CTRL_Duty(float ch1,float ch2,float ch3,float ch4)//2ms
{
	static u8 ctrl_2_cnt;
	float dT;
	
	//判断是否输出
	flag.landed = flag.thr_low;  //临时简单处理
	gf.out_weight = (flag.landed==1)?0:1;
	
	if(!flag.thr_low)//油门非低
	{
		if(gf.out_weight_slow<1)
		{
			gf.out_weight_slow += 1.5f *0.002f;//
		}
		else
		{
			gf.out_weight_slow = 1;
		}
	}
	else
	{
		gf.out_weight_slow = 0 ;
	}
	


	
/*=====================================================================================================================
						角度环
=====================================================================================================================*/	
	ctrl_2_cnt++;
	ctrl_2_cnt%=3;
	
	if(ctrl_2_cnt==0)//6ms
	{	
		//周期时间
		dT = 0.000001f *(GetSysTime_us() - ctrl_2_time);
		//期望角度
		float stick_val_rol = (0.002f *(+ch1)) *MAX_ROL_ANGLE;
		float stick_val_pit = (0.002f *(-ch2)) *MAX_PIT_ANGLE;
		
		float stick_val_rol_2;
		float stick_val_pit_2;
		
		headless_mode(0 ,stick_val_rol,stick_val_pit,&(stick_val_rol_2),&(stick_val_pit_2));//无头解算
					
		ctrl_2.exp_rol =  stick_val_rol_2;
		ctrl_2.exp_pit =  stick_val_pit_2;
		ctrl_2.exp_yaw = 0;
		//反馈角度
		ctrl_2.fb_rol = imu_data.rol;
		ctrl_2.fb_pit = imu_data.pit;
		ctrl_2.fb_yaw = -LIMIT(ctrl_1.exp_yaw_i_comp,-20,20);//imu_data.yaw; exp - (fb) = ctrl_1.exp_yaw_i_comp
		//角度环控制
		CTRL_2(dT,gf.out_weight_slow,&ctrl_2); //角度环
		//输出限幅
		ctrl_2.out_rol = LIMIT(ctrl_2.out_rol,-MAX_ROL_ANGLE,MAX_ROL_ANGLE);
		ctrl_2.out_pit = LIMIT(ctrl_2.out_pit,-MAX_PIT_ANGLE,MAX_PIT_ANGLE);
		ctrl_2.out_yaw = LIMIT(ctrl_2.out_yaw,-MAX_YAW_ANGLE,MAX_YAW_ANGLE);
		
		ctrl_2_time = GetSysTime_us();
	}
/*=====================================================================================================================
						角速度环
=====================================================================================================================*/	
	//周期
	dT = 0.000001f *(GetSysTime_us() - ctrl_1_time);
	//期望角速度
	ctrl_1.exp_rol = 8 *ctrl_2.out_rol *(0.25f + 0.75f *(ABS(ctrl_2.out_rol)/(MAX_ROL_ANGLE *arg_2_rol.kp)));//误差二次曲线     (0.002f *(+ch1)) *MAX_ROL_SPEED;//
	ctrl_1.exp_pit = 8 *ctrl_2.out_pit *(0.25f + 0.75f *(ABS(ctrl_2.out_pit)/(MAX_PIT_ANGLE *arg_2_pit.kp)));//误差二次曲线		(0.002f *(-ch2)) *MAX_PIT_SPEED;//
	ctrl_1.exp_yaw = my_deadzone(((0.002f *(+ch4))),0,0.1f) *MAX_YAW_SPEED;																	 //注释A处
	
	//因yaw无角度环，所以这里进行一个积分补偿，用于锁定机头
	if(!flag.landed)
	{
		ctrl_1.exp_yaw_i_comp += (ctrl_1.exp_yaw - (-sensor.Gyro_deg.z)) *dT;//该补偿值作为外环误差计算
		//ctrl_1.exp_yaw_i_comp = LIMIT(ctrl_1.exp_yaw_i_comp,-100,100);            
		if(ctrl_1.exp_yaw_i_comp>180)   
		{
			ctrl_1.exp_yaw_i_comp -=360;
		}			
		else if(ctrl_1.exp_yaw_i_comp<-180)
		{
			ctrl_1.exp_yaw_i_comp +=360;
		}
		
	}	
	ctrl_1.exp_yaw += 2 *ctrl_2.out_yaw *(0.25f + 0.75f *(ABS(ctrl_2.out_yaw)/(MAX_YAW_ANGLE *arg_2_yaw.kp)));//注释A处有赋值，非累加
	
	//限制最大期望角速度
	ctrl_1.exp_rol = LIMIT(ctrl_1.exp_rol,-MAX_ROL_SPEED,MAX_ROL_SPEED);
	ctrl_1.exp_pit = LIMIT(ctrl_1.exp_pit,-MAX_PIT_SPEED,MAX_PIT_SPEED);
	ctrl_1.exp_yaw = LIMIT(ctrl_1.exp_yaw,-MAX_YAW_SPEED,MAX_YAW_SPEED);
	
	//反馈角速度
	ctrl_1.fb_rol =  sensor.Gyro_deg.x;
	ctrl_1.fb_pit = -sensor.Gyro_deg.y;
	ctrl_1.fb_yaw = -sensor.Gyro_deg.z;
	
	

	
	//角速度环控制
	CTRL_1(dT,gf.out_weight_slow,&ctrl_1); //角速度环
	//输出限幅
	ctrl_1.out_rol = gf.out_weight *LIMIT(ctrl_1.out_rol,-1000,1000);
	ctrl_1.out_pit = gf.out_weight *LIMIT(ctrl_1.out_pit,-1000,1000);

	ctrl_1.out_yaw = gf.out_weight *LIMIT(ctrl_1.out_yaw,-1000,1000);
	
	ctrl_1_time = GetSysTime_us();
	
/*=====================================================================================================================
						油门环
=====================================================================================================================*/	
	//周期
	dT = 0.000001f *(GetSysTime_us() - ctrl_0_time);
	
	//期望速度
	if(ch3>0)
	{
		exp_hspeed = 0.002f *(+ch3) *MAX_ZSPEED_UP;
	}
	else
	{
		exp_hspeed = 0.002f *(+ch3) *MAX_ZSPEED_DN;
	}
	
	//模拟反馈
	LPF_1(2,dT,exp_hspeed,&exp_hspeed_lpf);
	
	//油门值
	thr = (+ch3) + 500;	
	//油门控制
	CTRL_0(dT,gf.out_weight,thr,exp_hspeed,exp_hspeed_lpf,&thr_value);
	
	thr_value = LIMIT(thr_value,0,800);//油门限制最大80%
	
	ctrl_0_time = GetSysTime_us();
	
/*=====================================================================================================================
						控制量输出
=====================================================================================================================*/	

	out_rol_curve = ctrl_1.out_rol;//my_pow_2_curve(ctrl_1.out_rol ,0.55,1000);
	out_pit_curve = ctrl_1.out_pit;//my_pow_2_curve(ctrl_1.out_pit ,0.55,1000);
	out_yaw_curve = ctrl_1.out_yaw;//my_pow_2_curve(ctrl_1.out_yaw ,0.55,1000);
	
	motor_ctrl(0.002f,(s16)out_rol_curve,(s16)out_pit_curve,(s16)out_yaw_curve,(s16)thr_value);
}


/*
      机头
   m2     m1
     \   /
      \ /
      / \
     /   \
   m3     m4
      屁股
*/
s16 motor[MOTOR_NUM];
void motor_ctrl(float dT,s16 ct_val_rol,s16 ct_val_pit,s16 ct_val_yaw,s16 ct_val_thr)
{
	u8 i;
	motor[m1] = ct_val_thr -ct_val_rol +ct_val_pit +ct_val_yaw;
	motor[m2] = ct_val_thr +ct_val_rol +ct_val_pit -ct_val_yaw;
	motor[m3] = ct_val_thr +ct_val_rol -ct_val_pit +ct_val_yaw;
	motor[m4] = ct_val_thr -ct_val_rol -ct_val_pit -ct_val_yaw;
	
	for(i=0;i<MOTOR_NUM;i++)
	{
		if(fly_ready)
		{
			if(flag.thr_low == 1)
			{
				motor[i] = LIMIT(motor[i],0,100);
			}
			else
			{
				motor[i] = LIMIT(motor[i],100,1000);
			}
		}
		else
		{
			motor[i] = 0;
		}
		
	}
//	motor[m4]=100;
	motor_out(motor);
	
}

/******************* (C) COPYRIGHT 2016 ANO TECH *****END OF FILE************/




