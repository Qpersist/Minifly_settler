#ifndef __ANO_DRV_UART1_H__
#define __ANO_DRV_UART1_H__

#include "include.h"

extern u8 RxState3;

void ANO_UART1_Init(u32 br_num);
void ANO_UART1_IRQ(void);

void ANO_UART1_Put_Char(unsigned char DataToSend);
void ANO_UART1_Put_String(unsigned char *Str);
void ANO_UART1_Put_Buf(unsigned char *DataToSend , u8 data_num);

#endif
