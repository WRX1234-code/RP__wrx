#include "board_protocol.h"
#include "string.h"
#include <stdbool.h>
#include "crc.h"
#include "usart.h"

C_Board_Tx_Pkt_t C_Board_Tx_Pkt = {
	.SOF = 0xA6,

};

C_Board_Rx_Info_t C_Board_Rx_Info;

uint8_t C_Board_TxBuf[58];


bool C_Board_Tx_Data(C_Board_Tx_Pkt_t* C_Board_Tx_Pkt)
{
	memcpy(C_Board_TxBuf, &C_Board_Tx_Pkt, sizeof(C_Board_Tx_Pkt));
	
	Append_CRC8_Check_Sum(C_Board_TxBuf, 3);
	Append_CRC16_Check_Sum(C_Board_TxBuf, sizeof(C_Board_Tx_Pkt));
	
	if(HAL_UART_Transmit_DMA(&huart6,C_Board_TxBuf,sizeof(C_Board_Tx_Pkt)) == HAL_OK)
	{
		return true;
	}
	
	return false;
}


bool C_Board_Rx_Data(C_Board_Rx_Info_t* C_Board_Rx_Info,uint8_t *rxBuf)
{
	if(rxBuf[0] == 0xA7)
	{
		if(Verify_CRC8_Check_Sum(rxBuf, 3) == true)
		{
			if(Verify_CRC16_Check_Sum(rxBuf, sizeof(C_Board_Rx_Info_t)) == true)
			{
				memcpy(&C_Board_Rx_Info, rxBuf, sizeof(C_Board_Rx_Info_t));
			
				return true;
			}
		}
	}
	return false;
}

