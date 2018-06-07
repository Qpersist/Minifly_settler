#include "Receive.h"
//#include "control.h"
#include "nrf24l01.h"
#include "ANO_Data.h"


#define ReceiveFrameHeaderH	0xAA
#define ReceiveFrameHeaderL	0xAA

#define FrameHeaderH_Addr		0u
#define FrameHeaderL_Addr		1u
#define FuncWord_Addr	2u
#define THR_Addr     	4u
#define YAW_Addr	 	6u
#define ROL_Addr		8u
#define PIT_Addr		10u


T_RC_Data 			g_t_Rc_D_CMS;			//遥控通道数据
void g_v_ReceiveData(void)
{
	uint8_t FrameHeader[2] = {0,0};
	uint8_t FuncWord = 0;
	static uint8_t flag_Lock = 0;
	FrameHeader[0] = g_u8a32_NRF24L01RXDataBuf_CMD[FrameHeaderH_Addr];
	FrameHeader[1] = g_u8a32_NRF24L01RXDataBuf_CMD[FrameHeaderL_Addr];
	
	FuncWord = g_u8a32_NRF24L01RXDataBuf_CMD[FuncWord_Addr];
	
	if((FrameHeader[0] != ReceiveFrameHeaderH) || (FrameHeader[1] != ReceiveFrameHeaderL))
	{
		return;
	}
	
	switch(FuncWord)
	{
		case 0x01:
			break;
		case 0x03:
			break;
		case 0x02:
			flag.NS = 2;
			g_t_Rc_D_CMS.THROTTLE = (((uint16_t)g_u8a32_NRF24L01RXDataBuf_CMD[THR_Addr] << 8) \
								  + (uint16_t)g_u8a32_NRF24L01RXDataBuf_CMD[THR_Addr + 1])-2000-20;	
			
		  g_t_Rc_D_CMS.YAW      = ((uint16_t)g_u8a32_NRF24L01RXDataBuf_CMD[YAW_Addr] << 8) \
								  + (uint16_t)g_u8a32_NRF24L01RXDataBuf_CMD[YAW_Addr + 1];

			g_t_Rc_D_CMS.ROLL     = ((uint16_t)g_u8a32_NRF24L01RXDataBuf_CMD[ROL_Addr] << 8) \
								  + (uint16_t)g_u8a32_NRF24L01RXDataBuf_CMD[ROL_Addr + 1];
//			g_t_Rc_D_CMS.ROLL     =g_t_Rc_D_CMS.ROLL;
			g_t_Rc_D_CMS.PITCH    = ((uint16_t)g_u8a32_NRF24L01RXDataBuf_CMD[PIT_Addr] << 8) \
								  + (uint16_t)g_u8a32_NRF24L01RXDataBuf_CMD[PIT_Addr + 1];		
//			g_t_Rc_D_CMS.PITCH=3000-g_t_Rc_D_CMS.PITCH+14+35;
			break;
		default :
			break;
	}
//	g_t_Rc_C_CMS.ARMED = 1;
	if((g_t_Rc_D_CMS.THROTTLE <= 1100) && (g_t_Rc_D_CMS.ROLL <= 1150))//(flag_Lock == 0) && 
	{
		flag_Lock = 1;
//		g_t_Rc_C_CMS.ARMED = 1;
//		LED1(OFF);
//		LED2(OFF);
//		LED3(OFF);
//		LED4(OFF);
	}
	if((g_t_Rc_D_CMS.THROTTLE <= 1100) && (g_t_Rc_D_CMS.ROLL>= 1950))
	{
		flag_Lock = 0;
//		g_t_Rc_C_CMS.ARMED = 0;
//		LED1(ON);
//		LED2(ON);
//		LED3(ON);
//		LED4(ON);
	}
}



















