#include "nrf24l01.h"
//#include "led.h"
#include "Receive.h"
//#include "usart.h"
uint8_t NRF24L01_RXDATA[RX_PLOAD_WIDTH];//nrf24l01���յ�������
uint8_t NRF24L01_TXDATA[RX_PLOAD_WIDTH];//nrf24l01��Ҫ���͵�����
u8  TX_ADDRESS[TX_ADR_WIDTH]= {0xAA,0xBB,0xCC,0x00,0x01};	//���ص�ַ
u8  RX_ADDRESS[RX_ADR_WIDTH]= {0xAA,0xBB,0xCC,0x00,0x01};	//���յ�ַ

volatile uint8_t g_u8a32_NRF24L01RXDataBuf_CMD[RX_PLOAD_WIDTH];//nrf24l01���յ�������
volatile uint8_t g_u8a32_NRF24L01TXDataBuf_CMD[TX_PLOAD_WIDTH];//nrf24l01���յ�������
uint8_t g_u8_NRF24L01RXDataBufLen_CMD = 0u;

/*
*****************************************************************
* д�Ĵ���
*****************************************************************
*/
uint8_t NRF_Write_Reg(uint8_t reg, uint8_t value)
{
	uint8_t status;
	ANO_SPI_CSN_L();					  /* ѡͨ���� */
	status = ANO_SPI_RW(reg);  /* д�Ĵ�����ַ */
	ANO_SPI_RW(value);		  /* д���� */
	ANO_SPI_CSN_H();					  /* ��ֹ������ */
  return 	status;
}
/*
*****************************************************************
* ���Ĵ���
*****************************************************************
*/
uint8_t NRF_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;
	ANO_SPI_CSN_L();					  /* ѡͨ���� */
	ANO_SPI_RW(reg);			  /* д�Ĵ�����ַ */
	reg_val = ANO_SPI_RW(0);	  /* ��ȡ�üĴ����������� */
	ANO_SPI_CSN_H();					  /* ��ֹ������ */
    return 	reg_val;
}
/*
*****************************************************************
*
* д������
*
*****************************************************************
*/
uint8_t NRF_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
	uint8_t i;
	uint8_t status;
	ANO_SPI_CSN_L();				        /* ѡͨ���� */
	status = ANO_SPI_RW(reg);	/* д�Ĵ�����ַ */
	for(i=0; i<uchars; i++)
	{
		ANO_SPI_RW(pBuf[i]);		/* д���� */
	}
	ANO_SPI_CSN_H();						/* ��ֹ������ */
    return 	status;	
}
/*
*****************************************************************
* ��������
*****************************************************************
*/
uint8_t NRF_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
	uint8_t i;
	uint8_t status;
	ANO_SPI_CSN_L();						/* ѡͨ���� */
	status = ANO_SPI_RW(reg);	/* д�Ĵ�����ַ */
	for(i=0; i<uchars; i++)
	{
		pBuf[i] = ANO_SPI_RW(0); /* ��ȡ�������� */ 	
	}
	ANO_SPI_CSN_H();						/* ��ֹ������ */
    return 	status;
}
/*
*****************************************************************
* д���ݰ�
*****************************************************************
*/
void g_v_NrfTxPacket(uint8_t * tx_buf, uint8_t len)
{	
	SPI_CE_L();		 //StandBy Iģʽ	
	NRF_Write_Buf(WR_TX_PLOAD, tx_buf, len); 			 // װ������	
	SPI_CE_H();		 //�ø�CE���������ݷ���
}
void NRF_TxPacket_AP(uint8_t * tx_buf, uint8_t len)
{	
	SPI_CE_L();		 //StandBy Iģʽ	
	NRF_Write_Buf(0xa8, tx_buf, len); 			 // װ������
	SPI_CE_H();		 //�ø�CE
}
u8 Nrf24l01_Check(void)
{ 
	u8 buf1[5]; 
	u8 i; 
	/*д��5���ֽڵĵ�ַ. */ 
	NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,5); 
	/*����д��ĵ�ַ */ 
	NRF_Read_Buf(TX_ADDR,buf1,5); 
	/*�Ƚ�*/ 
	for(i=0;i<5;i++) 
	{ 
		if(buf1[i]!=TX_ADDRESS[i]) 
			break; 
	} 
	if(i==5)
		return SUCCESS ; //MCU��NRF�ɹ����� 
	else
		return ERROR ; //MCU��NRF���������� 
}
void Nrf24l01_Init(u8 model, u8 ch)
{
	SPI_CE_L();
	NRF_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,RX_ADDRESS,RX_ADR_WIDTH);	//дRX�ڵ��ַ 
	NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,TX_ADR_WIDTH); 		//дTX�ڵ��ַ  
	NRF_Write_Reg(NRF_WRITE_REG+EN_AA,0x01); 													//ʹ��ͨ��0���Զ�Ӧ�� 
	NRF_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);											//ʹ��ͨ��0�Ľ��յ�ַ 
	NRF_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);											//�����Զ��ط����ʱ��:500us;����Զ��ط�����:10�� 
	NRF_Write_Reg(NRF_WRITE_REG+RF_CH,ch);														//����RFͨ��ΪCHANAL
	NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f); 												//����TX�������,0db����,2Mbps,���������濪��
	//NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x07); 												//����TX�������,0db����,1Mbps,���������濪��
/////////////////////////////////////////////////////////
	if(model==1)				//RX
	{
		NRF_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);								//ѡ��ͨ��0����Ч���ݿ�� 
		NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);   		 // IRQ�շ�����жϿ���,16λCRC,������
	}
	else if(model==2)		//TX
	{
		NRF_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);								//ѡ��ͨ��0����Ч���ݿ�� 
		NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);   		 // IRQ�շ�����жϿ���,16λCRC,������
	}
	else if(model==3)		//RX2
	{
		NRF_Write_Reg(FLUSH_TX,0xff);
		NRF_Write_Reg(FLUSH_RX,0xff);
		NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);   		 // IRQ�շ�����жϿ���,16λCRC,������
		
		ANO_SPI_RW(0x50);
		ANO_SPI_RW(0x73);
		NRF_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
		NRF_Write_Reg(NRF_WRITE_REG+0x1d,0x06);
	}
	else								//TX2
	{
		NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);   		 // IRQ�շ�����жϿ���,16λCRC,������
		NRF_Write_Reg(FLUSH_TX,0xff);
		NRF_Write_Reg(FLUSH_RX,0xff);
		
		ANO_SPI_RW(0x50);
		ANO_SPI_RW(0x73);
		NRF_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
		NRF_Write_Reg(NRF_WRITE_REG+0x1d,0x06);
	}
	SPI_CE_H();
}
////////////////////////////////////////////////////////////////////////////////
void Nrf_Check_Event(void)
{
	u8 sta = NRF_Read_Reg(NRF_READ_REG + NRFRegSTATUS);
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	if(sta & (1<<RX_DR))
	{
		u8 rx_len = NRF_Read_Reg(R_RX_PL_WID);
		if(rx_len<33)
		{
			NRF_Read_Buf(RD_RX_PLOAD,(uint8_t *)g_u8a32_NRF24L01RXDataBuf_CMD,rx_len);// read receive payload from RX_FIFO buffer
//			g_v_Uart1PutBuf(g_u8a32_NRF24L01RXDataBuf_CMD,rx_len);
			g_v_ReceiveData();
//			LED4(ON);
		}
		else 
		{
//			LED4(OFF);
			NRF_Write_Reg(FLUSH_RX,0xff);//��ջ�����
		}
	}
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	if(sta & (1<<TX_DS))
	{
//		LED3_turn();
	}
	else
	{
//		LED3(OFF);
	}
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	if(sta & (1<<MAX_RT))
	{
		if(sta & 0x01)	//TX FIFO FULL
		{
			NRF_Write_Reg(FLUSH_TX,0xff);
		}
	}
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	NRF_Write_Reg(NRF_WRITE_REG + NRFRegSTATUS, sta);
}
