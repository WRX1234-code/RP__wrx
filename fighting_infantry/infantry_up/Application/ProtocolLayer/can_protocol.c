#include "can_protocol.h"

/**
 *  @brief  CAN1 接收数据
 */
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case 0x201:
			Fric_Up_Motor.rx(&Fric_Up_Motor,rxBuf);
		  break;
		
		case 0x202:
			Fric_L_Motor.rx(&Fric_L_Motor,rxBuf);
		  break;
		
		case 0x203:
			Fric_R_Motor.rx(&Fric_R_Motor,rxBuf);
		  break;
		
		case 0x11:
			Pitch_Motor.rx(&Pitch_Motor,rxBuf);
      break;
		
		default:
			break;
	}
}
/**
 *  @brief  CAN2 接收数据
 */
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	
	switch (canId)
	{
		
		default:
			break;
	}
}
