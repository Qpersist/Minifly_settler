/******************** (C) COPYRIGHT 2014 ANO Tech ***************************
 * 作者		 ：匿名科创
 * 文件名  ：ANO_Drv_SPI.cpp
 * 描述    ：SPI
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/
#include "include.h"
#include "ANO_Drv_SPI.h"


void ANO_SPI_Init(void)
{
	SPI_InitTypeDef SPI_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	 
	/*ÅäÖÃ SPI_NRF_SPIµÄ SCK,MISO,MOSIÒý½Å£¬GPIOA^5,GPIOA^6,GPIOA^7 */ 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //¸´ÓÃ¹¦ÄÜ 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/*ÅäÖÃSPI_NRF_SPIµÄCEÒý½Å£¬ºÍSPI_NRF_SPIµÄ CSN Òý½Å:*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	ANO_SPI_CSN_H();
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //Ë«ÏßÈ«Ë«¹¤ 
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master; //Ö÷Ä£Ê½ 
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; //Êý¾Ý´óÐ¡8Î» 
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; //Ê±ÖÓ¼«ÐÔ£¬¿ÕÏÐÊ±ÎªµÍ 
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //µÚ1¸ö±ßÑØÓÐÐ§£¬ÉÏÉýÑØÎª²ÉÑùÊ±¿Ì 
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; //NSSÐÅºÅÓÉÈí¼þ²úÉú 
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; //8·ÖÆµ£¬9MHz 
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; //¸ßÎ»ÔÚÇ° 
	SPI_InitStructure.SPI_CRCPolynomial = 7; 
	SPI_Init(SPI2, &SPI_InitStructure); 
	/* Enable SPI1 */ 
	SPI_Cmd(SPI2, ENABLE);
	
}

u8 ANO_SPI_RW(u8 dat) 
{ 
	/* 当 SPI发送缓冲器非空时等待 */ 
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET); 
	/* 通过 SPI2发送一字节数据 */ 
	SPI_I2S_SendData(SPI2, dat); 
	/* 当SPI接收缓冲器为空时等待 */ 
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET); 
	/* Return the byte read from the SPI bus */ 
	return SPI_I2S_ReceiveData(SPI2); 
}

void ANO_SPI_CSN_H(void)
{
	GPIO_SetBits(GPIOA, SPI_Pin_CSN);
}

void ANO_SPI_CSN_L(void)
{
	GPIO_ResetBits(GPIOA, SPI_Pin_CSN);
}



/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
