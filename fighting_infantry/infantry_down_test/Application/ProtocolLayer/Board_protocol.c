#include "board_protocol.h"
#include "string.h"
#include <stdbool.h>
#include "crc.h"
#include "usart.h"

uint16_t Board_cnt;


D_Board_Tx_Pkt_t D_Board_Tx_Pkt = {
	.SOF = 0xA6,

};

D_Board_Rx_Info_t D_Board_Rx_Info;

uint8_t D_Board_TxBuf[68];


bool D_Board_Tx_Data(D_Board_Tx_Pkt_t* D_Board_Tx_Pkt)
{
	memcpy(D_Board_TxBuf, &D_Board_Tx_Pkt, sizeof(D_Board_Tx_Pkt));
	
	Append_CRC8_Check_Sum(D_Board_TxBuf, 3);
	Append_CRC16_Check_Sum(D_Board_TxBuf, sizeof(D_Board_Tx_Pkt));
	
	if(HAL_UART_Transmit_DMA(&huart1,D_Board_TxBuf,sizeof(D_Board_Tx_Pkt)) == HAL_OK)
	{
		return true;
	}
	
	return false;
}


bool D_Board_Rx_Data(D_Board_Rx_Info_t* D_Board_Rx_Info,uint8_t *rxBuf)
{
	if(rxBuf[0] == 0xA6)
	{
		if(Verify_CRC8_Check_Sum(rxBuf, 3) == true)
		{
			if(Verify_CRC16_Check_Sum(rxBuf, sizeof(D_Board_Rx_Info_t)) == true)
			{
				memcpy(&D_Board_Rx_Info, rxBuf, sizeof(D_Board_Rx_Info_t));
			  Board_cnt = 0;
				return true;
			}
		}
	}
	return false;
}

void D_Board_Heart_Beat(void)
{
	if(D_Board_Tx_Data(&D_Board_Tx_Pkt) == true)
	{
		Board_cnt ++;
	}
	if(Board_cnt >= OFFLINE_CNT_MAX)
	{
		Board_cnt = OFFLINE_CNT_MAX;
	}
}

/**
 *	@brief	在串口5中解析遥控数据协议
 */
void USART5_rxDataHandler(uint8_t *rxBuf)
{
	D_Board_Rx_Data(&D_Board_Rx_Info,rxBuf);
}

