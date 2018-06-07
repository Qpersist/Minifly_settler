/******************** (C) COPYRIGHT 2016 ANO Tech ***************************
 * ����		 �������ƴ�
 * �ļ���  ��ANO_Voltage.c
 * ����    ����ѹ���
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
**********************************************************************************/

#include "ANO_Voltage.h"
#include "ANO_Drv_ADC.h"

s16 voltage = 4000;//��λ 1mv

void voltage_check()
{
	voltage += 0.1f *(2 *(3300 *ADC_ConvertedValue[0]/4096) - voltage);
	
	if(voltage<3400)
	{
		flag.low_power = 6;
	}
	else if(voltage<3500)
	{
		flag.low_power = 5;
	}
	else if(voltage<3600)
	{
		flag.low_power = 4;
	}
	else if(voltage<3700)
	{
		flag.low_power = 3;
	}
	else if(voltage<3800)
	{
		flag.low_power = 2;
	}
	else if(voltage<4100)
	{
		flag.low_power = 1;
	}
	else
	{
		flag.low_power = 0;//��������4.1v
	}
	
	///////////////////////////
	
	if(flag.low_power>=5)
	{
		if(LED_warn==0)
		{
			LED_warn = 1;
		}
	}
	else if(flag.low_power<4)
	{
		if(LED_warn==1)
		{
			LED_warn = 0;
		}
	}
		
		
	
}



/******************* (C) COPYRIGHT 2016 ANO TECH *****END OF FILE************/

