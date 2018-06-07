#ifndef __ANO_RC_H
#define __ANO_RC_H

#include "stm32f10x.h"
#include "include.h"
#include "ANO_Drv_MPU6050.h"

typedef struct
{
	u16 s_cnt;
	u8 s_now_times;
	u8 s_state;
}_stick_f_c_st;


extern u8 fly_ready,NS;
extern u16 RX_CH[CH_NUM];
extern s16 CH_N[];

void RC_duty(float dT);
void fail_safe_check(void);

void stick_function(float dT);
	
#endif

