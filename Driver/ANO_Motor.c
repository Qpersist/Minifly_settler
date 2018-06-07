/******************** (C) COPYRIGHT 2016 ANO Tech ***************************
 * 作者		 ：匿名科创
 * 文件名  ：ANO_Motor.c
 * 描述    ：电机pwm输出
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
*****************************************************************************/
#include "ANO_Motor.h"
#include "mymath.h"

#define ACCURACY 1000 //0.001
#define Motor_PORT_PWM12	GPIOA
#define Motor_PORT_PWM34	GPIOB	

#define PWM1	GPIO_Pin_1
#define PWM2	GPIO_Pin_0
#define PWM3	GPIO_Pin_7
#define PWM4	GPIO_Pin_6

volatile uint16_t g_u16_Pwm1MC_CMS =00u;
volatile uint16_t g_u16_Pwm2MC_CMS =00u;
volatile uint16_t g_u16_Pwm3MC_CMS =00u;
volatile uint16_t g_u16_Pwm4MC_CMS =00u;


static void PWM_GPIO_Config(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;	 
	/* TIM2 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM4, ENABLE);//PCLK1¾­¹ý2±¶Æµºó×÷ÎªTIM2µÄÊ±ÖÓÔ´µÈÓÚ72MHz
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE); /* GPIOA clock enable */
	/*GPIOA Configuration: TIM2 channel 1 and 2 as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin =  PWM1 | PWM2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // ¸´ÓÃÍÆÍìÊä³ö
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(Motor_PORT_PWM12, &GPIO_InitStructure);
	
	/*GPIOB Configuration: TIM4 channel 3 and 4 as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin =  PWM3 | PWM4;
	GPIO_Init(Motor_PORT_PWM34, &GPIO_InitStructure);
//	GPIO_ResetBits(Motor_PORT_PWM12,PWM1|PWM2);
//	GPIO_ResetBits(Motor_PORT_PWM34,PWM3|PWM4);
}


static void PWM_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	uint16_t PrescalerValue = 0;


	/* Time base configuration */		 
	TIM_TimeBaseStructure.TIM_Period = 999;       //µ±¶¨Ê±Æ÷´Ó0¼ÆÊýµ½3599£¬¼´Îª3600´Î£¬ÎªÒ»¸ö¶¨Ê±ÖÜÆÚ
	PrescalerValue = (uint16_t) (SystemCoreClock / (24000*ACCURACY)) - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;	//pwm时钟分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;	//ÉèÖÃÊ±ÖÓ·ÖÆµÏµÊý£º²»·ÖÆµ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //ÏòÉÏ¼ÆÊýÄ£Ê½

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);	

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //ÅäÖÃÎªPWMÄ£Ê½1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
	TIM_OCInitStructure.TIM_Pulse = 0;	   //ÉèÖÃÌø±äÖµ£¬µ±¼ÆÊýÆ÷¼ÆÊýµ½Õâ¸öÖµÊ±£¬µçÆ½·¢ÉúÌø±ä
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //µ±¶¨Ê±Æ÷¼ÆÊýÖµÐ¡ÓÚCCR1_ValÊ±Îª¸ßµçÆ½
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);	 //ÅäÖÃÍ¨µÀ1
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);	 //ÅäÖÃÍ¨µÀ1
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel2 */
	TIM_OCInitStructure.TIM_Pulse = 0;	  //ÉèÖÃÍ¨µÀ2µÄµçÆ½Ìø±äÖµ£¬Êä³öÁíÍâÒ»¸öÕ¼¿Õ±ÈµÄPWM
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);	  //ÅäÖÃÍ¨µÀ2
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);	  //ÅäÖÃÍ¨µÀ2
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM2, ENABLE);	// Ê¹ÄÜTIM2ÖØÔØ¼Ä´æÆ÷ARR
	TIM_ARRPreloadConfig(TIM4, ENABLE);	// Ê¹ÄÜTIM4ÖØÔØ¼Ä´æÆ÷ARR
	TIM_Cmd(TIM2, ENABLE);	///* TIM2 enable counter */
	TIM_Cmd(TIM4, ENABLE);	//* TIM4 enable counter */   
}


void g_v_PwmInit(void)
{
	PWM_GPIO_Config();
	PWM_Mode_Config();	
}
void pwm_out_init()
{
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  				TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_StructInit(&GPIO_InitStructure);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_OCStructInit(&TIM_OCInitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	uint16_t PrescalerValue = 0;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	/* Compute the prescaler value */
	PrescalerValue = (uint16_t) (SystemCoreClock / (24000*ACCURACY)) - 1;
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 999;		//
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;	//pwm时钟分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		//向上计数
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	/* PWM1 Mode configuration: Channel */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;//初始占空比为0
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);	
	
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);	
	
	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_Cmd(TIM2, ENABLE);

}



void motor_out(s16 pwm[MOTOR_NUM])
{
	u8 i;
	for(i=0;i<MOTOR_NUM;i++)
	{
		pwm[i] = LIMIT(pwm[i],0,1000);
	}
	
	TIM2->CCR1 = (u16)pwm[0] ; 
	TIM2->CCR2 = (u16)pwm[1] ;	
	TIM4->CCR1 = (u16)pwm[2] ; 
	TIM4->CCR2 = (u16)pwm[3] ;

}

/******************* (C) COPYRIGHT 2016 ANO TECH *****END OF FILE************/



