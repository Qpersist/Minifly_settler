#ifndef _Receive_H_
#define _Receive_H_
#include "stm32f10x.h"
typedef struct int16_rcget
{
	int16_t ROLL;
	int16_t PITCH;
	int16_t THROTTLE;
	int16_t YAW;
	int16_t AUX1;
	int16_t AUX2;
	int16_t AUX3;
	int16_t AUX4;
	int16_t AUX5;
	int16_t AUX6;
}T_RC_Data;
extern T_RC_Data 			g_t_Rc_D_CMS;	
void g_v_ReceiveData(void);







#endif


