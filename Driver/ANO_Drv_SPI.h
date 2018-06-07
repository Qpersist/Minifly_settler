#ifndef __ANO_DRV_SPI_H__
#define __ANO_DRV_SPI_H__

#include "stm32f10x.h"
#define SPI_CE_H()   GPIO_SetBits(GPIOA,GPIO_Pin_8)
#define SPI_CE_L()   GPIO_ResetBits(GPIOA,GPIO_Pin_8)
	void ANO_SPI_Init(void);
	u8 ANO_SPI_RW(u8 dat);
	void ANO_SPI_CE_H(void);
	void ANO_SPI_CE_L(void);
	void ANO_SPI_CSN_H(void);
	void ANO_SPI_CSN_L(void);

#endif










