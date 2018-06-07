#include "ANO_Drv_Uart.h"
#include <stdarg.h>
#include "ANO_Data_Transfer.h"
char Rec_Bluetooth_Buf[8];
uint8_t j = 0;


#define USART1_PORT_CLK	RCC_APB2Periph_GPIOA
#define USART1_PORT		GPIOA
#define USART1_TX		GPIO_Pin_9
#define USART1_RX		GPIO_Pin_10

#define USART2_PORT_CLK	RCC_APB2Periph_GPIOA
#define USART2_PORT		GPIOA
#define USART2_TX		GPIO_Pin_2
#define USART2_RX		GPIO_Pin_3


u8 Rx_Buf[32];
u8 Rx_Adr=0;
/*
 * º¯ÊýÃû£ºUSART1_Config
 * ÃèÊö  £ºUSART1 GPIO ÅäÖÃ,¹¤×÷Ä£Ê½ÅäÖÃ¡£115200 8-N-1
 * ÊäÈë  £ºÎÞ
 * Êä³ö  : ÎÞ
 * µ÷ÓÃ  £ºÍâ²¿µ÷ÓÃ
 */
void USART1_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	/* config USART1 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | USART1_PORT_CLK, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	/* USART1 GPIO config */
	/* Configure USART1 Tx (PA.09) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = USART1_TX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART1_PORT, &GPIO_InitStructure);    
	/* Configure USART1 Rx (PA.10) as input floating */
	GPIO_InitStructure.GPIO_Pin = USART1_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(USART1_PORT, &GPIO_InitStructure);
	  
	/* USART1 mode config */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure); 
	  	/* Ê¹ÄÜ´®¿Ú1½ÓÊÕÖÐ¶Ï */
			
	USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //???????
	USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK??????????->???
	USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //?????????????
	USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //?????????????SCLK??
	
	USART_Init(USART1, &USART_InitStructure);
	USART_ClockInit(USART1, &USART_ClockInitStruct);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);


	USART_Cmd(USART1, ENABLE);

//	USART_ClearFlag( USART1, USART_FLAG_TC );
//	USART_ClearITPendingBit(USART1,USART_IT_RXNE);
}

/*
 * º¯ÊýÃû£ºUSART2_Config
 * ÃèÊö  £ºUSART2 GPIO ÅäÖÃ,¹¤×÷Ä£Ê½ÅäÖÃ¡£115200 8-N-1
 * ÊäÈë  £ºÎÞ
 * Êä³ö  : ÎÞ
 * µ÷ÓÃ  £ºÍâ²¿µ÷ÓÃ
 */
void USART2_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/* config USART2 clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(USART2_PORT_CLK, ENABLE);
	
	/* USART2 GPIO config */
	/* Configure USART2 Tx (PA2) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = USART2_TX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART2_PORT, &GPIO_InitStructure);    
	/* Configure USART2 Rx (PA3) as input floating */
	GPIO_InitStructure.GPIO_Pin = USART2_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(USART2_PORT, &GPIO_InitStructure);
	  
	/* USART2 mode config */
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);  

  	/* Ê¹ÄÜ´®¿Ú2½ÓÊÕÖÐ¶Ï */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	USART_Cmd(USART2, ENABLE);
	USART_ClearFlag( USART2, USART_FLAG_TC );
	USART_ClearITPendingBit(USART2,USART_IT_RXNE);
}

/*
 * º¯ÊýÃû£ºfputc
 * ÃèÊö  £ºÖØ¶¨Ïòc¿âº¯Êýprintfµ½USART
 * ÊäÈë  £ºÎÞ
 * Êä³ö  £ºÎÞ
 * µ÷ÓÃ  £ºÓÉprintfµ÷ÓÃ
 */
//int fputc(int ch, FILE *f)
//{
//	/* ½«PrintfÄÚÈÝ·¢Íù´®¿Ú */
//	USART_SendData(USART2, (unsigned char) ch);
////	while (!(USART2->SR & USART_FLAG_TXE));
//	while( USART_GetFlagStatus(USART2,USART_FLAG_TC)!= SET);	
//	return (ch);
//}

/*
 * º¯ÊýÃû£ºitoa
 * ÃèÊö  £º½«ÕûÐÎÊý¾Ý×ª»»³É×Ö·û´®
 * ÊäÈë  £º-radix =10 ±íÊ¾10½øÖÆ£¬ÆäËû½á¹ûÎª0
 *         -value Òª×ª»»µÄÕûÐÎÊý
 *         -buf ×ª»»ºóµÄ×Ö·û´®
 *         -radix = 10
 * Êä³ö  £ºÎÞ
 * ·µ»Ø  £ºÎÞ
 * µ÷ÓÃ  £º±»USART_printf()µ÷ÓÃ
 */
static char *itoa(int value, char *string, int radix)
{
	int     i, d;
	int     flag = 0;
	char    *ptr = string;
	
	/* This implementation only works for decimal numbers. */
	if (radix != 10)
	{
	    *ptr = 0;
	    return string;
	}
	
	if (!value)
	{
	    *ptr++ = 0x30;
	    *ptr = 0;
	    return string;
	}
	
	/* if this is a negative value insert the minus sign. */
	if (value < 0)
	{
	    *ptr++ = '-';
	
	    /* Make the value positive. */
	    value *= -1;
	}
	
	for (i = 10000; i > 0; i /= 10)
	{
	    d = value / i;
	
	    if (d || flag)
	    {
	        *ptr++ = (char)(d + 0x30);
	        value -= (d * i);
	        flag = 1;
	    }
	}
	
	/* Null terminate the string. */
	*ptr = 0;
	
	return string;

} /* NCL_Itoa */

/*
 * º¯ÊýÃû£ºUSART_printf
 * ÃèÊö  £º¸ñÊ½»¯Êä³ö£¬ÀàËÆÓÚC¿âÖÐµÄprintf£¬µ«ÕâÀïÃ»ÓÐÓÃµ½C¿â
 * ÊäÈë  £º-USARTx ´®¿ÚÍ¨µÀ£¬ÕâÀïÖ»ÓÃµ½ÁË´®¿Ú2£¬¼´USART2
 *		     -Data   Òª·¢ËÍµ½´®¿ÚµÄÄÚÈÝµÄÖ¸Õë
 *			   -...    ÆäËû²ÎÊý
 * Êä³ö  £ºÎÞ
 * ·µ»Ø  £ºÎÞ 
 * µ÷ÓÃ  £ºÍâ²¿µ÷ÓÃ
 *         µäÐÍÓ¦( USART2, "\r\n this is a demo \r\n" );
 *            		 USART_printf( USART2, "\r\n %d \r\n", i );
 *            		 USART_printf( USART2, "\r\n %s \r\n", j );
 */

void USART_printf(USART_TypeDef* USARTx, uint8_t *Data,...)
{
	const char *s;
	int d;   
	char buf[16];
	
	va_list ap;
	va_start(ap, Data);
	
	while ( *Data != 0)     // ÅÐ¶ÏÊÇ·ñµ½´ï×Ö·û´®½áÊø·û
	{				                          
		if ( *Data == 0x5c )  //'\'
	{									  
	switch ( *++Data )
	{
		case 'r':							          //»Ø³µ·û
			USART_SendData(USARTx, 0x0d);
			Data ++;
		break;
		
		case 'n':							          //»»ÐÐ·û
			USART_SendData(USARTx, 0x0a);	
			Data ++;
		break;
		
		default:
			Data ++;
		break;
	}			 
	}
	else if ( *Data == '%')
	{									  //
	switch ( *++Data )
	{				
		case 's':										  //×Ö·û´®
			s = va_arg(ap, const char *);
	for ( ; *s; s++) 
	{
		USART_SendData(USARTx,*s);
		while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
	}
		Data++;
		break;
	
	case 'd':										//Ê®½øÖÆ
	d = va_arg(ap, int);
	itoa(d, buf, 10);
	for (s = buf; *s; s++) 
	{
		USART_SendData(USARTx,*s);
		while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
	}
	Data++;
	break;
		 default:
				Data++;
		    break;
	}		 
	} /* end of else if */
	else USART_SendData(USARTx, *Data++);
	while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
	}
} 

void clearBluetooth_Buff(void)
{
	USART_ITConfig(USART_DEBUG, USART_IT_RXNE, DISABLE);	 //È¡ÏûUSART3µÄ½ÓÊÕÖÐ¶Ï

    for(j=0;j<Bluetooth_Max;j++)    //½«»º´æÄÚÈÝÇåÁã
    {
		Rec_Bluetooth_Buf[j] = 0;
	}
    j = 0;                    //½ÓÊÕ×Ö·û´®µÄÆðÊ¼´æ´¢Î»ÖÃ
	USART_ITConfig(USART_DEBUG, USART_IT_RXNE, ENABLE); //´ò¿ªUSART3µÄ½ÓÊÕÖÐ¶Ï
}
u8 TxBuffer[256];
u8 TxCounter=0;
u8 count=0;
//static u8 RxBuffer[50];
//static u8 RxState = 0;

void g_v_Uart1_IRQ(void)
{	
	

	if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)//??!????if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)???
    {
        u8 com_data = USART1->DR;
			USART_ClearFlag(USART3,USART_IT_ORE);
    }
		
	//·¢ËÍÖÐ¶Ï
	if((USART1->SR & (1<<7))&&(USART1->CR1 & USART_CR1_TXEIE))//if(USART_GetITStatus(USART1,USART_IT_TXE)!=RESET)
	{
		USART1->DR = TxBuffer[TxCounter++]; //Ð´DRÇå³ýÖÐ¶Ï±êÖ¾          
		if(TxCounter == count)
		{    
			USART1->CR1 &= ~USART_CR1_TXEIE;		//¹Ø±ÕTXEÖÐ¶Ï
			//USART_ITConfig(USART1,USART_IT_TXE,DISABLE);
		}
	}
	
	//½ÓÊÕÖÐ¶Ï
if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	 {
		u8 com_data = USART1->DR;
		ANO_DT_Data_Receive_Prepare(com_data);
	 }
}

void g_v_Uart1PutBuf(unsigned char *DataToSend , uint8_t data_num)
{
	uint8_t i=0;
	for(i=0;i<data_num;i++)
	{
		TxBuffer[count++] = *(DataToSend+i);
	}
	if(!(USART1->CR1 & USART_CR1_TXEIE))
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE); 
}


