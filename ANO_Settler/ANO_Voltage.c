/******************** (C) COPYRIGHT 2016 ANO Tech ***************************
 * 作者		 ：匿名科创
 * 文件名  ：ANO_Voltage.c
 * 描述    ：电压监测
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/

#include "ANO_Voltage.h"
#include "ANO_Drv_ADC.h"

s16 voltage = 4000;//单位 1mv

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
		flag.low_power = 0;//充满大于4.1v
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

