/******************** (C) COPYRIGHT 2014 ANO Tech ***************************
 * 作者		 ：匿名科创
 * 文件名  ：ANO_Drv_LED.cpp
 * 描述    ：LED
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/
#include "ANO_Drv_LED.h"
#include "ANO_RC.h"

void ANO_LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);      /*使能SWD 禁用JTAG*/
	RCC_APB2PeriphClockCmd(ANO_RCC_LED_0,ENABLE);

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = ANO_Pin_LED_0|LED1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void ANO_LED_0_OFF(void)
{
	//GPIO_ResetBits(ANO_GPIO_LED_0, ANO_Pin_LED_0);	
	ANO_GPIO_LED_0->BRR = ANO_Pin_LED_0;
}

void ANO_LED_0_ON(void)
{
	//GPIO_SetBits(ANO_GPIO_LED_0, ANO_Pin_LED_0);	
	ANO_GPIO_LED_0->BSRR = ANO_Pin_LED_0;	
}

u16 led_accuracy = 20;//该时间应与LED_Duty()调用周期相同
float on_time;

void LED_1ms_DRV()
{
	static u16 led_cnt;
	
	if(led_cnt<on_time)
	{
		ANO_LED_0_ON();
	}
	else
	{
		ANO_LED_0_OFF();
	}
	
	if(++led_cnt>=led_accuracy)
	{
		led_cnt = 0;
	}

}
u8 LED_warn;
//亮-灭 为一组
void LED_Dyty(float dT)
{
	switch(LED_warn)
	{
		case 0:
			if(!fly_ready)
			{
				led_breath(dT,1500);
			}
			else
			{
				led_flash(dT,2,100,100,500);//调用周期（s），组数，亮时间(ms)，灭时间(ms)，组间隔 ,ms>led_accuracy;
			}			
		break;
			
		case 1://没电
			
			led_flash(dT,1,200,200,0);//调用周期（s），组数，亮时间(ms)，灭时间(ms)，组间隔 ,ms>led_accuracy;
		break;
		
		case 2://校准gyro
			if(led_flash(dT,6,100,100,0)) //调用周期（s），组数，亮时间(ms)，灭时间(ms)，组间隔 ,ms>led_accuracy;
			{
				LED_warn = 0;
			}
		break;
		
		case 3://校准acc
			if(led_flash(dT,12,100,100,0)) //调用周期（s），组数，亮时间(ms)，灭时间(ms)，组间隔 ,ms>led_accuracy;
			{
				LED_warn = 0;
			}
			
		break;
		
		default:break;
	}



}



u8 led_breath(float dT,u16 T)// 一次半程的时间，单位ms
{
	u8 f = 0;
	static u8 dir;
	switch(dir)
	{
		case 0:
			on_time += safe_div(led_accuracy,(T/(dT*1000)),0);
			if(on_time>20)
			{
				dir = 1;
			}
		break;
		case 1:
			on_time -= safe_div(led_accuracy,(T/(dT*1000)),0);
			if(on_time<0)
			{
				dir = 0;
				f = 1;
			}
		break;
			
		default:
			dir = 0;
		break;
			
	}
	return (f);

}
//亮-灭 为一组
//调用周期（s），组数，亮时间(ms)，灭时间(ms)，组间隔 ,ms>led_accuracy;
u8 led_flash(float dT,u16 group_n,u16 on_ms,u16 off_ms,u16 group_dT_ms)
{
	u8 f=0;
	static u16 ms_cnt;
	static u16 group_n_cnt;
	
	if(group_n_cnt < group_n)   //组数没到
	{
		if(ms_cnt<on_ms)
		{
			on_time = 20;
		}
		else if(ms_cnt<(on_ms+off_ms))
		{
			on_time = 0;
		}
		if(ms_cnt>=(on_ms+off_ms))
		{
			group_n_cnt ++;
			ms_cnt = 0;
		}
	}
	else						//进入组间隔
	{
		if(ms_cnt<group_dT_ms)
		{
			on_time = 0;
		}
		else
		{
			group_n_cnt = 0;
			ms_cnt = 0;
			f = 1; //流程完成1次
		}
	}
	
	ms_cnt += (dT*1000);        //计时
	return (f); //0，未完成，1完成
}
/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

