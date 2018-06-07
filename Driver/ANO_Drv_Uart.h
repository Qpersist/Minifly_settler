#ifndef __ANO_DRV_UART_H
#define	__ANO_DRV_UART_H

#include "stm32f10x.h"
#include <stdio.h>

#define  Bluetooth_Max  9
#define USART_DEBUG USART1
extern char Rec_Bluetooth_Buf[8];
extern uint8_t j;

void USART1_Config(void);
void USART2_Config(void);
int fputc(int ch, FILE *f);
void USART_printf(USART_TypeDef* USARTx, uint8_t *Data,...);
void clearBluetooth_Buff(void);

void g_v_Uart1_IRQ(void);
void g_v_Uart1PutBuf(unsigned char *DataToSend , u8 data_num);
#endif /* __USART_H */
