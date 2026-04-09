#include "vision_protocol.h"
#include "Board_protocol.h" 
#include "string.h"
#include <stdbool.h>
#include "crc.h"
#include "usart.h"
#include "usbd_cdc_if.h"

uint16_t vision_cnt;

ElectricalToVisionFrame vision_tx_frame = {
	.SOF = 0xA5,
};

VisionToElectricalFrame vision_rx_frame;

uint8_t Vision_TxBuf[54];

bool Vision_Tx_data(ElectricalToVisionFrame* vision_tx_frame)
{
	memcpy(Vision_TxBuf, vision_tx_frame, sizeof(ElectricalToVisionFrame));
	
	Append_CRC8_Check_Sum(Vision_TxBuf, 2);
	Append_CRC16_Check_Sum(Vision_TxBuf, sizeof(ElectricalToVisionFrame));
	
	if(CDC_Transmit_FS(Vision_TxBuf,sizeof(ElectricalToVisionFrame)) == USBD_OK)
	{
    
		return true;
	}
	
	return false;
}


bool Vision_Rx_Data(VisionToElectricalFrame* vision_rx_frame,uint8_t *rxBuf)
{
	if(rxBuf[0] == 0xA5)
	{
		if(Verify_CRC8_Check_Sum(rxBuf, 2) == true)
		{
			if(Verify_CRC16_Check_Sum(rxBuf, sizeof(VisionToElectricalFrame)) == true)
			{
				memcpy(vision_rx_frame, rxBuf, sizeof(VisionToElectricalFrame));
				vision_cnt = 0;
				return true;
			}
			
		}
	}
	return false;
}

void Vision_heart_beat(void)
{
	vision_cnt ++;
	if(vision_cnt >= 70)
	{
		vision_cnt = 70;
		C_Board_Tx_Pkt.vision_state = 0;
	}
//	else if(((vision_rx_frame.all_flags>>0) & 1) == 0)
//	{
//		C_Board_Tx_Pkt.vision_state = 0;
//	}
	else{
	  C_Board_Tx_Pkt.vision_state = 1;
	}
	  
	
}

